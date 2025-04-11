from pathlib import Path
import threading
import time
import rclpy
import subprocess
from rclpy.node import Node
from cartographer_ros_msgs.msg import SubmapList

from cartographer_tuner.utils.terminal_runnable import TerminalRunnable
from cartographer_tuner.tools.exceptions import ExternalToolRunException
from cartographer_tuner.submap_analyzer.submap_subscriber_node import SubmapSubscriberNode
from cartographer_tuner.submap_analyzer.submap import Submap


class FirstSubmapCartographerLauncher(TerminalRunnable):
    """Launcher for saving the first completed submap from a bag file using Cartographer.
    
    This utility:
    1. Launches Cartographer online node
    2. Plays the bag file
    3. Subscribes to submap updates
    4. Waits for the first submap to be completed (detected when a second submap appears)
    5. Saves the first submap and terminates
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
            "rate",
            float,
            required=False,
            default=1.0,
            help="Rate at which to play back the bag file"
        )
        cls.register_parameter(
            "start_offset",
            float,
            required=False,
            default=0.0,
            help="Skip to this time (seconds) in the bag file"
        )
        cls.register_parameter(
            "output_path",
            str,
            required=True,
            help="Path to save the first submap"
        )

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._done = False
        self._submap_subscriber = None
        self._submap_list_lock = threading.Lock()
        self._first_submap_saved = False
        self._cartographer_process = None
        self._bag_play_process = None
        self._first_submap = None
        
        # Ensure output directory exists
        Path(self._output_path).parent.mkdir(parents=True, exist_ok=True)

    def _on_submap_list_update(self, submap_list: SubmapList) -> None:
        """Handle incoming submap list updates.
        
        When there are at least 2 submaps, it means the first submap is complete.
        We then query and save that first submap.
        """
        with self._submap_list_lock:
            # Check if we have at least 2 submaps
            if not hasattr(submap_list, 'submap') or len(submap_list.submap) < 2:
                return
            
            if self._first_submap is None:
                print("WARN: first submap is expected to be defined")
                return

            self._first_submap_saved = True
            self._done = True
            self._first_submap.save(self._output_path)
            print(f"First submap saved to {self._output_path}")

    def _on_submap_data_update(self, trajectory_id: int, submap_index: int, submap: Submap) -> None:
        """Handle incoming submap data. We don't need to do anything here."""
        if trajectory_id == 0 and submap_index == 0:
            self._first_submap = submap

    def launch_cartographer(self):
        """Launch cartographer node."""
        try:
            # Build the ROS command to launch cartographer node
            cmd = [
                "ros2", "run", "cartographer_ros", "cartographer_node",
                "-configuration_directory", str(self._configuration_directory),
                "-configuration_basename", self._configuration_basename,
            ]
            
            # Start the cartographer process
            self._cartographer_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            print(f"Launched cartographer with command: {' '.join(cmd)}")
            
            # Start a thread to monitor the output
            threading.Thread(
                target=self._monitor_process_output,
                args=(self._cartographer_process, "Cartographer"),
                daemon=True
            ).start()
            
        except Exception as e:
            raise ExternalToolRunException(f"Failed to launch cartographer: {str(e)}")

    def play_bag_file(self):
        """Play the bag file."""
        try:
            # Build the ROS command to play the bag file
            cmd = [
                "ros2", "bag", "play",
                str(self._bag_filename),
                "--rate", str(self._rate),
                "--start-offset", str(self._start_offset),
                "--clock",
                "--disable-keyboard-controls"
            ]
            
            # Start the bag play process
            self._bag_play_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            print(f"Playing bag file with command: {' '.join(cmd)}")
            
            # Start a thread to monitor the output
            threading.Thread(
                target=self._monitor_process_output,
                args=(self._bag_play_process, "BagPlay"),
                daemon=True
            ).start()
            
        except Exception as e:
            raise ExternalToolRunException(f"Failed to play bag file: {str(e)}")

    def _monitor_process_output(self, process, name):
        """Monitor process output."""
        if not process:
            return
            
        while process.poll() is None:
            line = process.stdout.readline()
            if line:
                print(f"{name}: {line.strip()}")
                
        # Check if process exited with an error
        if process.returncode != 0:
            print(f"{name} process exited with code {process.returncode}")
            for line in process.stderr.readlines():
                print(f"{name} error: {line.strip()}")

    def initialize_submap_subscriber(self):
        """Initialize the submap subscriber."""
        self._submap_subscriber = SubmapSubscriberNode(
            list_update_callback=self._on_submap_list_update,
            submap_update_callback=self._on_submap_data_update
        )

    def run(self):
        """Main entry point for the utility."""
        try:
            rclpy.init()  
            self.launch_cartographer()
            self.play_bag_file()

            

            self.initialize_submap_subscriber()
            
            
            while not self._done:
                self._submap_subscriber.process_events(timeout_sec=0.1)
                
                # Check if any process has ended
                if (self._cartographer_process and self._cartographer_process.poll() is not None) or \
                   (self._bag_play_process and self._bag_play_process.poll() is not None):
                    print("One of the processes has ended. Exiting.")
                    break
                
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("Interrupted by user")
        except Exception as e:
            print(f"Error: {e}")
        finally:
            if self._submap_subscriber:
                self._submap_subscriber.shutdown()
            
            if self._cartographer_process and self._cartographer_process.poll() is None:
                print("Terminating cartographer process...")
                self._cartographer_process.terminate()
                
            if self._bag_play_process and self._bag_play_process.poll() is None:
                print("Terminating bag play process...")
                self._bag_play_process.terminate()
                
            rclpy.shutdown()
            print("FirstSubmapCartographerLauncher finished") 