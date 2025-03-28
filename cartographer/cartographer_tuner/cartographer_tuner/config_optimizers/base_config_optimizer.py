from abc import abstractmethod
from typing import List

from cartographer_tuner.utils.terminal_runnable import TerminalRunnable


class BaseOptimizer(TerminalRunnable):
    """Abstract base class for all optimizers."""
   
    @abstractmethod
    def available_metrics(self) -> List[str]:
        pass

    @classmethod
    def _register_params(cls):
        cls.register_parameter(
            name="bag_filename",
            param_type=str,
            required=True,
            help="Path to input bag file"
        )
        cls.register_parameter(
            name="config_dir",
            param_type=str,
            required=True,
            help="Directory containing Lua configs"
        )
        cls.register_parameter(
            name="config_basename",
            param_type=str,
            required=True,
            help="Base name of the Lua config file"
        )
        cls.register_parameter(
            name="metrics",
            param_type=str,
            required=True,
            help="Comma-separated list of metrics to evaluate",
            default=None
        )
        cls.register_parameter(
            name="skip_seconds",
            param_type=int,
            required=False,
            help="Seconds to skip from bag start",
            default=0
        )
