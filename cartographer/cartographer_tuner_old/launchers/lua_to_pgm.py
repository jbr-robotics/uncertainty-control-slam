#!/usr/bin/env python3

import os
import tempfile
from pathlib import Path
from typing import Dict, List, Optional, Any

import rclpy
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node as LaunchNode
from launch_ros.actions import SetRemap

from quality_estimator.launchers.base import BaseLauncher
from quality_estimator.launchers.offline_cartographer import OfflineCartographerLauncher
from quality_estimator.launchers.pbstream_to_pgm import PbstreamToPgmLauncher


class LuaToPgmLauncher(BaseLauncher):
    """Launcher for generating PGM maps from bag files using Cartographer with Lua configuration.
    
    This launcher combines the functionality of OfflineCartographerLauncher and PbstreamToPgmLauncher
    to generate a PGM map from a bag file using Cartographer with Lua configuration.
    
    The process is:
    1. Run Cartographer on the bag file to generate a pbstream file (in a temporary directory)
    2. Convert the pbstream file to a PGM map (at the specified output path)
    
    This launcher is useful for generating maps for metric calculators that operate on PGM maps.
    """
    
    @classmethod
    def _register_params(cls):
        # Register parameters from OfflineCartographerLauncher
        cls.register_parameter(
            "bag_filenames",
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
            "configuration_basenames",
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
    
    def __init__(self, **kwargs):
        """Initialize the launcher with the provided parameters."""
        super().__init__(**kwargs)
        
        # Create temporary directory for intermediate files
        self._tmp_dir = tempfile.mkdtemp(prefix="lua_to_pgm_tmp_")
        print(f"Created temporary directory for intermediate files: {self._tmp_dir}")
        
        # Get the base name of the bag file for naming the pbstream file
        bag_basename = Path(self._bag_filenames).stem
        
        # Initialize paths for intermediate files (in temporary directory)
        self._pbstream_path = Path(self._tmp_dir) / f"{bag_basename}.pbstream"
        
        # Initialize paths for output files
        self._map_filestem_path = Path(self._map_filestem)
        # Ensure the output directory exists
        self._map_filestem_path.parent.mkdir(parents=True, exist_ok=True)
        
        # The PGM and YAML files will be created at the specified output path
        self._pgm_path = Path(f"{self._map_filestem_path}.pgm")
        self._yaml_path = Path(f"{self._map_filestem_path}.yaml")
    
    def generate_launch_description(self) -> LaunchDescription:
        """Generate launch description for the Lua to PGM conversion.
        
        This method is not used directly, as we override the run method to run the launchers sequentially.
        """
        # This is a placeholder - we override the run method to run the launchers sequentially
        return LaunchDescription(self.get_base_description_elements())
    
    def run(self) -> None:
        """Run the Lua to PGM conversion process.
        
        This method runs the OfflineCartographerLauncher and PbstreamToPgmLauncher
        sequentially to generate a PGM map from a bag file.
        
        Raises:
            RuntimeError: If the conversion fails at any step
        """
        try:
            # Step 1: Run offline Cartographer to generate pbstream (in temporary directory)
            print(f"Running offline Cartographer to generate pbstream file: {self._pbstream_path}")
            
            # Create a dictionary of parameters for the OfflineCartographerLauncher
            cartographer_params = {
                "bag_filenames": str(self._bag_filenames),
                "configuration_directory": str(self._configuration_directory),
                "configuration_basenames": self._configuration_basenames,
                "skip_seconds": self._skip_seconds,
                "no_rviz": self._no_rviz,
                "rviz_config": self._rviz_config,
                "save_state_filename": str(self._pbstream_path), 
                "remap": self.remappings
            }
            
            # Create and run the OfflineCartographerLauncher
            cartographer_launcher = OfflineCartographerLauncher(**cartographer_params)
            
            try:
                cartographer_launcher.run()
            except Exception as e:
                raise RuntimeError(f"Failed to run offline Cartographer: {str(e)}")
            
            # Verify that the pbstream file was created
            if not self._pbstream_path.exists():
                raise RuntimeError(f"pbstream file was not created: {self._pbstream_path}")
            
            # Step 2: Convert pbstream to PGM map (at the specified output path)
            print(f"Converting pbstream to PGM map: {self._map_filestem_path}")
            
            pgm_launcher = PbstreamToPgmLauncher(
                pbstream_filename=str(self._pbstream_path),
                map_filestem=str(self._map_filestem_path),
                resolution=self._resolution
            )
            
            try:
                pgm_launcher.run()
            except Exception as e:
                raise RuntimeError(f"Failed to convert pbstream to PGM map: {str(e)}")
            
            # Verify that the map files were created
            if not self._pgm_path.exists():
                raise RuntimeError(f"PGM map file was not created: {self._pgm_path}")
            if not self._yaml_path.exists():
                raise RuntimeError(f"YAML metadata file was not created: {self._yaml_path}")
            
            print(f"Successfully generated PGM map: {self._pgm_path}")
            print(f"YAML metadata: {self._yaml_path}")
            
        except Exception as e:
            raise RuntimeError(f"Error in Lua to PGM conversion: {str(e)}")
        # finally:
        #     # Clean up temporary directory
        #     import shutil
        #     shutil.rmtree(self._tmp_dir)
        #     print(f"Cleaned up temporary directory: {self._tmp_dir}")
    
    def get_pgm_path(self) -> Path:
        """Get the path to the generated PGM map file.
        
        Returns:
            Path to the PGM map file
        """
        return self._pgm_path
    
    def get_yaml_path(self) -> Path:
        """Get the path to the generated YAML metadata file.
        
        Returns:
            Path to the YAML metadata file
        """
        return self._yaml_path


# Generate main function
main = LuaToPgmLauncher.generate_main()

if __name__ == "__main__":
    main()
