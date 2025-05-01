import json
import uuid
import shutil
import pickle
from pathlib import Path
from typing import Dict, Any, List, Type, Optional, Union

import numpy as np
from hyperopt import fmin, tpe, hp, Trials, STATUS_OK
from hyperopt.pyll import scope # For integer parameters

from cartographer_tuner.utils.terminal_runnable import TerminalRunnable
from cartographer_tuner.tools.combinations.submap_sampler_cartographer import SubmapSamplerCartographer
from cartographer_tuner.metrics.metric import Metric
from cartographer_tuner.core.cartographer_config_manager import CartographerConfigManager
from cartographer_tuner.submap_analyzer.submap import Submap 

from cartographer_tuner.metrics.calculators.pgm import (
    EnclosedAreasCalculator,
    CornerCountCalculator,
    OccupiedProportionCalculator,
    UnsureAreaProportionCalculator
)

METRIC_CALCULATOR_CLASSES: List[Type] = [
    EnclosedAreasCalculator,
    CornerCountCalculator,
    OccupiedProportionCalculator,
    UnsureAreaProportionCalculator
]
assert METRIC_CALCULATOR_CLASSES, "METRIC_CALCULATOR_CLASSES list cannot be empty."

PRIMARY_LOSS_METRIC_NAME = UnsureAreaProportionCalculator.UNSURE_AREA_PROPORTION
assert isinstance(PRIMARY_LOSS_METRIC_NAME, str) and PRIMARY_LOSS_METRIC_NAME, \
    "PRIMARY_LOSS_METRIC_NAME must be a non-empty string."


class SubmapHyperoptSearch(TerminalRunnable):
    """
    Runs Cartographer parameter optimization using hyperopt with strict error checking.

    This version prioritizes correctness by failing early if preconditions are not met
    or intermediate steps produce unexpected results (e.g., invalid metric values).
    It uses SubmapSamplerCartographer, CartographerConfigManager, loads Submap from .pkl,
    evaluates intensity data with multiple metrics, and optimizes based on a single,
    validated primary loss metric.
    """

    @classmethod
    def _register_params(cls):
        """Registers parameters required for the hyperopt search process."""
        cls.register_parameter("search_space_def", str, True, "JSON string or file path defining the parameter search space")
        cls.register_parameter("config_dir", str, True, "Directory containing the base Cartographer Lua configuration files")
        cls.register_parameter("config_basename", str, True, "Base name of the main Lua config file")
        cls.register_parameter("bag_filename", str, True, "Path to the input ROS bag file")
        cls.register_parameter("output_root", Path, True, "Root directory where trial outputs will be stored")
        cls.register_parameter("max_evals", int, True, "Maximum number of parameter sets to evaluate", default=50)
        cls.register_parameter("samples", int, True, "Number of submaps to generate per trial evaluation", default=1)
        cls.register_parameter("rate", float, False, "Playback rate for the bag file", default=1.0)
        cls.register_parameter("skip_begin", float, False, "Seconds to skip at the start of the bag file", default=0.0)
        cls.register_parameter("skip_end", float, False, "Seconds to skip at the end of the bag file", default=0.0)

    def run(self):
        """Executes the hyperopt optimization process with top-level error handling."""
        try:
            search_space_config = self._load_search_space_config()
            hyperopt_space = self._create_hyperopt_space(search_space_config)

            self._output_root.mkdir(parents=True, exist_ok=True)

            trials = Trials()

            print(f"\n==> Starting hyperopt optimization with max_evals={self._max_evals}")
            print(f"    Base config directory: {self._config_dir}")
            print(f"    Base config basename: {self._config_basename}")
            print(f"    Primary loss metric for optimization: '{PRIMARY_LOSS_METRIC_NAME}'")
            print(f"    Strict checking enabled: Trials will fail on invalid intermediate results.")

            best = fmin(
                fn=self._objective_function,
                space=hyperopt_space,
                algo=tpe.suggest,
                max_evals=self._max_evals,
                trials=trials
            )

            print("\n==> Optimization finished!")
            best_trial_params = trials.argmin

            if best_trial_params is not None:
                print("Best parameters found (according to primary loss):")
                best_params_readable = self._get_readable_params(best_trial_params, search_space_config)
                print(json.dumps(best_params_readable, indent=2))

                # Display details from the best trial entry
                best_trial_entry = trials.best_trial
                if best_trial_entry and 'result' in best_trial_entry:
                    print(f"\nBest trial details (Trial Index: {best_trial_entry.get('tid', 'N/A')}):")
                    best_loss = best_trial_entry['result'].get('loss')
                    assert best_loss is not None and np.isfinite(best_loss), \
                        f"Best trial loss is invalid: {best_loss}" # Should not happen if objective fn is strict
                    print(f"  Primary Loss ({PRIMARY_LOSS_METRIC_NAME}): {best_loss:.4f}")

                    if 'attachments' in best_trial_entry['result'] and 'mean_metrics' in best_trial_entry['result']['attachments']:
                         print("  Mean metrics for the best trial:")
                         mean_metrics_best = best_trial_entry['result']['attachments']['mean_metrics']
                         for name, value in mean_metrics_best.items():
                              assert value is not None and np.isfinite(value), \
                                  f"Invalid metric '{name}' found in best trial attachments: {value}" # Should not happen
                              print(f"    - {name}: {value:.4f}")
                else:
                     print("\nWarning: Could not retrieve full details dictionary for the best trial.") # Should not happen ifargmin worked
            else:
                 print("\nNo best trial found by hyperopt. Optimization might have failed or been interrupted.")

        except (AssertionError, ValueError, TypeError, FileNotFoundError, RuntimeError, ImportError) as e:
             print(f"\n[FATAL ERROR] Optimization setup or execution failed: {e}")
             import traceback
             traceback.print_exc()
        except Exception as e:
             print(f"\n[FATAL UNEXPECTED ERROR] Hyperopt optimization process failed: {e}")
             import traceback
             traceback.print_exc()


    def _objective_function(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Core objective function for a single hyperopt trial. Executes the
        simulation and evaluation pipeline with strict checks.

        Returns:
            Dictionary with 'loss', 'status', and attachments upon success.
            Dictionary with 'status': 'fail' and infinite loss upon any critical failure.
        """
        trial_id = '_'.join(
            f"{val}" for _, val in sorted(params.items())
        )
        output_path = self._output_root / f"{trial_id}"
        output_path.mkdir(parents=True, exist_ok=True)

        print(f"\n==> Starting Trial {trial_id} with params: {json.dumps(params)}")
        print(f"    Output path: {output_path}")

        try:
            # === Step 1: Prepare Configuration ===
            print("    Preparing config directory for trial...")
            config_dir_for_sampler = self._prepare_trial_config_directory(params, output_path)
            print(f"    Trial config directory prepared: {config_dir_for_sampler}")

            # === Step 2: Run Submap Sampling ===
            print("    Running SubmapSamplerCartographer...")
            # Exceptions during sampler run will be caught by the outer try...except
            sampler = SubmapSamplerCartographer(
                bag_filename=self._bag_filename,
                configuration_directory=str(config_dir_for_sampler),
                configuration_basename=self._config_basename,
                samples=self._samples,
                output_path=output_path,
                rate=self._rate,
                skip_begin=self._skip_begin,
                skip_end=self._skip_end,
            )
            sampler.run() # Assume this blocks and raises exception on failure
            print("    Submap sampling finished.")

            # === Step 3: Evaluate Generated Submaps ===
            print("    Evaluating generated submaps...")
            # This method now contains strict checks and will raise errors on invalid data/results
            mean_metrics = self._evaluate_submaps(output_path)
            print(f"    Mean metrics calculated successfully: {json.dumps({k: f'{v:.4f}' for k, v in mean_metrics.items()})}")

            # === Step 4: Determine Loss and Validate ===
            # Assert that the primary metric was actually calculated
            assert PRIMARY_LOSS_METRIC_NAME in mean_metrics, \
                f"Primary loss metric '{PRIMARY_LOSS_METRIC_NAME}' not found in calculated mean metrics."
            primary_loss = mean_metrics[PRIMARY_LOSS_METRIC_NAME]

            # Assert that the primary loss value is finite (not Inf or NaN)
            assert np.isfinite(primary_loss), \
                f"Primary loss metric '{PRIMARY_LOSS_METRIC_NAME}' has non-finite value: {primary_loss}"

            # === Step 5: Return Success Result ===
            result = {
                'loss': primary_loss,
                'status': STATUS_OK,
                'params': params,
                'trial_id': trial_id,
                'attachments': {'mean_metrics': mean_metrics}
            }
            return result

        except Exception as e:
            print(f"[Error] Trial {trial_id} failed: {e}")
            import traceback
            traceback.print_exc()
            return {
                'status': 'fail',
                'loss': float('inf'),
                'error': str(e),
                'params': params,
                'trial_id': trial_id,
            }


    def _prepare_trial_config_directory(self, params: Dict[str, Any], trial_output_path: Path) -> Path:
        """
        Prepares a dedicated config directory for a trial using CartographerConfigManager.
        Includes stricter checks for file operations and parameter application.

        Returns: Path to the prepared configuration subdirectory.
        Raises: FileNotFoundError, RuntimeError, AssertionError if setup fails.
        """
        config_subdir = trial_output_path / "configs"
        config_subdir.mkdir(parents=True, exist_ok=True)

        base_config_dir = Path(self._config_dir)
        base_config_file_path = base_config_dir / self._config_basename

        assert base_config_dir.is_dir(), f"Base config directory not found: {base_config_dir}"
        assert base_config_file_path.is_file(), f"Base config file '{self._config_basename}' not found in {base_config_dir}"

        # --- Copy Base Lua Files ---
        print(f"    Copying base Lua files from {base_config_dir} to {config_subdir}...")
        lua_files_to_copy = list(base_config_dir.glob("*.lua"))
        # Strict: Fail if no Lua files are found (assuming at least the base file should exist)
        assert lua_files_to_copy, f"No *.lua files found in base config directory: {base_config_dir}"
        copied_files_count = 0
        try:
            for lua_file in lua_files_to_copy:
                shutil.copy(str(lua_file), str(config_subdir))
                copied_files_count += 1
            # Optional strict check: Ensure all found files were copied
            assert copied_files_count == len(lua_files_to_copy), \
                 f"Failed to copy all {len(lua_files_to_copy)} Lua files to {config_subdir}"
            print(f"    Copied {copied_files_count} Lua file(s).")
        except Exception as copy_e:
            # Re-raise copy errors as RuntimeError to fail the trial
            raise RuntimeError(f"Failed during copy of base config files to {config_subdir}: {copy_e}")

        # --- Load, Modify, Save Config ---
        print(f"    Loading base config: {base_config_file_path}")
        config_manager = CartographerConfigManager()
        # Let exceptions from load() propagate
        config_manager.load(base_config_file_path)

        print(f"    Applying {len(params)} parameter(s) using CartographerConfigManager...")
        applied_count = 0
        # Strict: Fail if ANY parameter cannot be set
        for param_path, value in params.items():
            try:
                config_manager[param_path] = value
                applied_count += 1
            except Exception as set_e:
                raise RuntimeError(f"CartographerConfigManager failed to set parameter "
                                   f"'{param_path}' to value {value}: {set_e}")
        # Optional check (can be removed if the above loop guarantees it):
        assert applied_count == len(params), "Not all parameters were applied successfully by the loop logic."


        modified_config_path = config_subdir / self._config_basename
        print(f"    Saving modified config to: {modified_config_path}")
        # Let exceptions from save_to_file() propagate
        config_manager.save_to_file(modified_config_path)

        return config_subdir


    def _evaluate_submaps(self, trial_output_path: Path) -> Dict[str, float]:
        """
        Evaluates submaps from .pkl files with strict validation.

        Fails the trial (by raising Exception) if no submaps are found,
        if any submap fails to load or provides invalid data, or if any
        metric calculation yields a non-finite value.

        Returns: Dictionary mapping metric names to their mean finite values.
        """
        # --- Find and Validate Submap Files ---
        submap_files = sorted(list(trial_output_path.glob("submap_*.pkl")))
        # Strict: Fail if no submap files were generated by the sampler
        assert submap_files, f"No 'submap_*.pkl' files found in {trial_output_path}. Cannot evaluate."
        print(f"    Found {len(submap_files)} submap PKL files for evaluation.")

        # --- Process Submaps and Calculate Metrics ---
        all_metric_results: Dict[str, List[float]] = {}
        processed_submap_count = 0

        for i, pkl_file in enumerate(submap_files):
            print(f"      - Evaluating submap {i+1}/{len(submap_files)}: {pkl_file.name}")
            submap_obj = Submap.load(str(pkl_file))
            assert isinstance(submap_obj, Submap), f"Object loaded from {pkl_file} is not a Submap instance."

            map_data: np.ndarray = submap_obj.intensity
            assert isinstance(map_data, np.ndarray) and map_data.size > 0, \
                f"Intensity data invalid (not ndarray or empty) in {pkl_file.name}."

            # --- Run Metric Calculators ---
            for CalculatorClass in METRIC_CALCULATOR_CLASSES:
                calc_instance_name = CalculatorClass.__name__
                calculator = CalculatorClass(map_data=map_data)
                results_dict: Dict[str, Metric] = calculator.calculate()
                assert isinstance(results_dict, dict), \
                    f"Calculator {calc_instance_name} did not return a dictionary for {pkl_file.name}."

                # Strict: Validate and store results; fail on invalid metric values
                for metric_name, metric_obj in results_dict.items():
                     assert isinstance(metric_obj, Metric), \
                         f"Value for '{metric_name}' from {calc_instance_name} is not a Metric object."
                     metric_value = metric_obj.value
                     assert metric_value is not None and np.isfinite(metric_value), \
                         f"Metric '{metric_name}' from {calc_instance_name} has invalid value ({metric_value}) for {pkl_file.name}."

                     if metric_name not in all_metric_results:
                          all_metric_results[metric_name] = []
                     all_metric_results[metric_name].append(float(metric_value))

            processed_submap_count += 1

        assert processed_submap_count > 0, "No submaps were successfully processed and evaluated."
        assert processed_submap_count == len(submap_files), \
             f"Only {processed_submap_count}/{len(submap_files)} submaps processed without critical errors."

        # --- Calculate Mean Values ---
        mean_metrics: Dict[str, float] = {}
        print("    Calculating mean values across evaluated submaps...")
        all_expected_metric_names = set()
        for calculator_cls in METRIC_CALCULATOR_CLASSES:
            all_expected_metric_names.update(getattr(calculator_cls, 'METRIC_NAMES', [calculator_cls.__name__]))

        for metric_name in all_expected_metric_names:
            value_list = all_metric_results.get(metric_name)
            assert value_list is not None and len(value_list) == processed_submap_count, \
                f"Metric '{metric_name}' was not calculated for all {processed_submap_count} processed submaps."

            mean_value = np.mean(value_list)
            assert np.isfinite(mean_value), f"Mean value for metric '{metric_name}' is not finite: {mean_value}"
            mean_metrics[metric_name] = float(mean_value)
            print(f"      - {metric_name}: mean = {mean_value:.4f} (from {len(value_list)} values)")

        return mean_metrics


    def _load_search_space_config(self) -> Dict[str, Any]:
        """Loads and validates the search space definition."""
        path_or_json = self._search_space_def
        try:
            print("    Attempting to parse search space definition as JSON string.")
            config = json.loads(path_or_json)
        except json.JSONDecodeError as e:
            raise ValueError(f"Invalid JSON in search space definition: {e}") from e
        except FileNotFoundError:
             raise FileNotFoundError(f"Search space definition file not found: {path_or_json}")
        except Exception as e:
            raise ValueError(f"Error loading search space definition from '{path_or_json}': {e}")

        assert isinstance(config, dict), "Search space definition must be a JSON object (dictionary)."
        assert config, "Search space definition dictionary cannot be empty." # Ensure not empty
        return config


    def _create_hyperopt_space(self, config: Dict[str, Any]) -> Dict:
        """Creates the hyperopt search space with strict validation."""
        space = {}
        print("    Building hyperopt search space:")
        assert isinstance(config, dict) and config, "Input config must be a non-empty dictionary."

        for name, details in config.items():
            assert isinstance(details, dict), f"Definition for parameter '{name}' must be a dictionary."

            param_type = details.get("type")
            assert param_type, f"Missing 'type' key for parameter '{name}'."
            param_type = param_type.lower()
            label = name

            try:
                if param_type == "float":
                    assert "min" in details and "max" in details, f"Missing 'min' or 'max' for float parameter '{name}'."
                    min_val, max_val = float(details["min"]), float(details["max"])
                    assert min_val < max_val, f"min must be less than max for float parameter '{name}'."
                    space[name] = hp.uniform(label, min_val, max_val)
                    print(f"      - {name}: float (uniform, min={min_val}, max={max_val})")
                elif param_type == "int":
                    assert "min" in details and "max" in details, f"Missing 'min' or 'max' for int parameter '{name}'."
                    min_val, max_val = int(details["min"]), int(details["max"])
                    assert min_val <= max_val, f"min must be less than or equal to max for int parameter '{name}'."
                    space[name] = scope.int(hp.quniform(label, min_val, max_val, q=1))
                    print(f"      - {name}: int (quniform, min={min_val}, max={max_val})")
                elif param_type == "bool":
                    space[name] = hp.choice(label, [False, True])
                    print(f"      - {name}: bool (choice)")
                elif param_type == "choice":
                    assert "options" in details, f"Missing 'options' key for choice parameter '{name}'."
                    options = details["options"]
                    assert isinstance(options, list) and options, \
                        f"'options' for choice parameter '{name}' must be a non-empty list."
                    space[name] = hp.choice(label, options)
                    print(f"      - {name}: choice (options={options})")
                else:
                    raise ValueError(f"Unsupported parameter type '{param_type}' for parameter '{name}'.")

            except (KeyError, AssertionError) as e: # Catch assertion errors too
                raise ValueError(f"Invalid definition for parameter '{name}': {e}") from e
            except (TypeError, ValueError) as type_e:
                 raise ValueError(f"Invalid value type in definition for parameter '{name}': {type_e}") from type_e

        return space


    def _get_readable_params(self, best_param_set: Dict[str, Any], search_space_config: Dict[str, Any]) -> Dict[str, Any]:
        """Converts hyperopt's result format back to readable values with validation."""
        readable_params = {}
        assert isinstance(best_param_set, dict), "best_param_set must be a dictionary."
        assert isinstance(search_space_config, dict), "search_space_config must be a dictionary."

        for name, index_or_value in best_param_set.items():
            details = search_space_config.get(name)
            assert details is not None, f"Parameter '{name}' from hyperopt result not found in search space config."
            assert isinstance(details, dict), f"Search space definition for '{name}' is not a dictionary."

            param_type = details.get("type", "float").lower()

            try:
                if param_type == "bool":
                    assert index_or_value in [0, 1], f"Invalid index '{index_or_value}' for bool parameter '{name}'."
                    readable_params[name] = [False, True][int(index_or_value)]
                elif param_type == "choice":
                    options = details["options"]
                    assert isinstance(options, list), f"'options' key missing or not a list for choice param '{name}'."
                    idx = int(index_or_value)
                    assert 0 <= idx < len(options), \
                        f"Index {idx} out of bounds for options of choice parameter '{name}'."
                    readable_params[name] = options[idx]
                else: # float or int
                     # Basic type check (though hyperopt should return correct type)
                     assert isinstance(index_or_value, (int, float, np.number)), \
                          f"Expected numerical value for param '{name}', got {type(index_or_value).__name__}."
                     # Ensure int param is actually int (via scope.int)
                     if param_type == "int":
                          assert isinstance(index_or_value, (int, np.integer)), \
                               f"Expected int value for param '{name}', got {type(index_or_value).__name__}."
                     readable_params[name] = index_or_value
            except (KeyError, IndexError, ValueError, TypeError, AssertionError) as e:
                 raise RuntimeError(f"Failed to convert hyperopt result value for parameter '{name}': {e}") from e

        return readable_params
