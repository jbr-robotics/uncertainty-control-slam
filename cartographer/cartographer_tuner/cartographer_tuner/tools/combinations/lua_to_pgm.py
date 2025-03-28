import tempfile
from pathlib import Path

from cartographer_tuner.tools.ros.offline_cartographer_launcher import OfflineCartographerLauncher
from cartographer_tuner.tools.ros.pbstream_to_pgm_launcher import PbstreamToPgmLauncher
from cartographer_tuner.tools.base_tool import BaseTool
from cartographer_tuner.tools.exceptions import ExternalToolRunException

class LuaToPgmLauncher(BaseTool):
    """Launcher for generating PGM maps from bag files using Cartographer with Lua configuration.
    """
    
    @classmethod
    def _register_params(cls):
        cls.register_parameter(
            "bag_filename",
            str,
            required=True,
            help="Path to the bag file"
        )
        cls.register_parameter(
            "configuration_directory",
            str,
            required=True,
            help="Directory containing Lua configuration files"
        )
        cls.register_parameter(
            "configuration_basename",
            str,
            required=True,
            help="Base name of the Lua configuration file"
        )
        cls.register_parameter(
            "skip_seconds",
            int,
            required=False,
            default=0,
            help="Seconds to skip from the beginning of the bag file"
        )
        cls.register_parameter(
            "no_rviz",
            str,
            required=False,
            default="true",
            help="Disable RViz visualization"
        )
        cls.register_parameter(
            "rviz_config",
            str,
            required=False,
            help="Path to the RViz configuration file"
        )
        cls.register_parameter(
            "resolution",
            float,
            required=False,
            default=0.05,
            help="Resolution of the map in meters per pixel"
        )
        cls.register_parameter(
            "map_filestem",
            str,
            required=True,
            help="Path to save the output PGM and YAML files (without extension)"
        )

    def pbstream_path(self, dir: Path) -> Path:
        bag_basename = Path(self._bag_filename).stem
        return Path(dir) / f"{bag_basename}.pbstream"
    
    @property
    def pgm_path(self) -> Path:
        return Path(f"{self._map_filestem}.pgm")
    
    @property
    def yaml_path(self) -> Path:
        return Path(f"{self._map_filestem}.yaml")
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)        
        Path(self._map_filestem).parent.mkdir(parents=True, exist_ok=True)
    
    def run_offline_cartographer(self, directory: Path) -> None:
        cartographer_params = {
            "bag_filename": str(self._bag_filename),
            "configuration_directory": str(self._configuration_directory),
            "configuration_basename": self._configuration_basename,
            "skip_seconds": self._skip_seconds,
            "no_rviz": self._no_rviz,
            "rviz_config": self._rviz_config,
            "save_state_filename": str(self.pbstream_path(directory)), 
        }
            
        cartographer_launcher = OfflineCartographerLauncher(**cartographer_params)

        try:
            cartographer_launcher.run()
        except Exception as e:
            raise ExternalToolRunException(f"Failed to run offline Cartographer: {str(e)}")

        if not self.pbstream_path(directory).exists():
            raise ExternalToolRunException(f"pbstream file was not created: {self.pbstream_path(directory)}")

    def run_pbstream_to_pgm(self, directory: Path) -> None:
        pgm_launcher = PbstreamToPgmLauncher(
                pbstream_filename=str(self.pbstream_path(directory)),
                map_filestem=str(self._map_filestem),
                resolution=self._resolution
            )
            
        try:
            pgm_launcher.run()
        except Exception as e:
            raise ExternalToolRunException(f"Failed to convert pbstream to PGM map: {str(e)}")
        
        if not self.pgm_path.exists():
            raise ExternalToolRunException(f"PGM map file was not created: {self.pgm_path}")
        if not self.yaml_path.exists():
            raise ExternalToolRunException(f"YAML metadata file was not created: {self.yaml_path}")

    def run(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            self.run_offline_cartographer(Path(tmp_dir)) 
            self.run_pbstream_to_pgm(Path(tmp_dir))
