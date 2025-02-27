from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Dict, Type, Optional, ClassVar, List
import argparse
import rclpy
from launch import LaunchService, LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap

@dataclass
class LauncherParameter:
    """Metadata for launcher parameters."""
    type: Type
    required: bool = True
    help: str = ""
    default: Any = None
    launch_arg: bool = True  # Whether to create a launch argument for this parameter
    # Store any additional argparse parameters
    argparse_kwargs: Dict[str, Any] = field(default_factory=dict)

class BaseLauncher(ABC):
    """Abstract base class for all launchers.
    
    This class provides:
    1. Automatic CLI argument parsing
    2. Parameter registration and validation
    3. Common launcher interface
    
    To create a new launcher:
    1. Inherit from BaseLauncher
    2. Register parameters using register_parameter classmethod
    3. Implement the run() method
    """
    
    # Class variable to store parameter definitions
    parameters: ClassVar[Dict[str, LauncherParameter]] = {}
    
    # Register remapping parameter by default
    parameters["remap"] = LauncherParameter(
        type=str,
        required=False,
        help="Topic remapping in format '/source_topic;/target_topic' where source_topic is the topic in the bag file and target_topic is the expected topic name (e.g., '/base_scan;/scan' to map from /base_scan to /scan). Can be specified multiple times.",
        default=None,
        argparse_kwargs={"action": "append"}
    )
    
    @classmethod
    def register_parameter(
        cls,
        name: str,
        param_type: Type,
        required: bool = True,
        help: str = "",
        default: Any = None,
        launch_arg: bool = True,
        **argparse_kwargs
    ) -> None:
        """Register a new parameter for the launcher.
        
        Args:
            name: Parameter name (will be used as CLI argument name)
            param_type: Python type for the parameter
            required: Whether the parameter is required
            help: Help text for the CLI argument
            default: Default value if not required
            launch_arg: Whether to create a launch argument for this parameter
            **argparse_kwargs: Additional keyword arguments to pass to argparse.add_argument()
        """
        cls.parameters[name] = LauncherParameter(
            type=param_type,
            required=required,
            help=help,
            default=default,
            launch_arg=launch_arg,
            argparse_kwargs=argparse_kwargs
        )
    
    @classmethod
    def generate_main(cls) -> callable:
        """Generate a main function for the launcher.
        
        Returns:
            Callable that serves as the entry point for the launcher
        """
        def main():
            # Create parser with class docstring as description
            parser = argparse.ArgumentParser(description=cls.__doc__)
            
            # Add arguments based on registered parameters
            for name, param in cls.parameters.items():
                # Combine basic parameters with additional argparse kwargs
                arg_params = {
                    "type": param.type,
                    "required": param.required,
                    "help": param.help,
                    "default": param.default,
                    **param.argparse_kwargs
                }
                parser.add_argument(f"--{name}", **arg_params)
            
            # Parse arguments
            args = parser.parse_args()
            
            # Create launcher instance with parsed args
            launcher = cls(**vars(args))
            
            # Run the launcher
            launcher.run()
        
        return main
    
    def __init__(self, **kwargs):
        """Initialize launcher with provided parameters.
        
        Args:
            **kwargs: Parameter values matching registered parameters
            
        Raises:
            ValueError: If required parameters are missing or invalid types provided
        """
        # Store remappings separately
        self.remappings = []
        if "remap" in kwargs and kwargs["remap"]:
            # Handle remappings from action='append' format
            for remap_str in kwargs["remap"]:
                print(f"Remapping: {remap_str}")
                try:
                    from_topic, to_topic = remap_str.split(';')
                    self.remappings.append((from_topic, to_topic))
                    print(f"Remapping: {from_topic} -> {to_topic}")
                except ValueError:
                    raise ValueError(
                        f"Invalid remapping format: {remap_str}. "
                        "Expected format: '/from_topic;/to_topic'"
                    )
            del kwargs["remap"]

        # Process other parameters
        for name, param in self.parameters.items():
            if name == "remap":
                continue
            if param.required and name not in kwargs:
                raise ValueError(f"Required parameter '{name}' not provided")
            
            # Get value, using default if not provided
            value = kwargs.get(name, param.default)
            
            # Validate type if value provided
            if value is not None and not isinstance(value, param.type):
                raise ValueError(
                    f"Parameter '{name}' must be of type {param.type.__name__}, "
                    f"got {type(value).__name__}"
                )
            
            # Store parameter as instance attribute
            setattr(self, f"_{name}", value)
    
    def get_base_description_elements(self) -> List:
        """Get common launch description elements.
        
        Returns:
            List of launch description elements including:
            - Launch arguments
            - Topic remappings
        """
        return self.generate_launch_arguments() + self.get_remappings()

    @abstractmethod
    def generate_launch_description(self) -> LaunchDescription:
        """Generate launch description for the launcher.
        
        Returns:
            LaunchDescription object defining the launch configuration
            
        Example implementation:
            description = self.get_base_description_elements() + [
                # Add your specific nodes and other elements here
            ]
            return LaunchDescription(description)
        """
        pass

    def run(self) -> None:
        """Execute the launcher using ROS2 launch system.
        
        This default implementation handles ROS2 initialization and shutdown.
        Override if different launch behavior is needed.
        """
        # Initialize ROS
        rclpy.init()
        
        try:
            # Create and run launch service
            launch_service = LaunchService()
            launch_service.include_launch_description(self.generate_launch_description())
            launch_service.run()
        finally:
            # Ensure ROS is shutdown
            rclpy.shutdown()

    def generate_launch_arguments(self) -> List[DeclareLaunchArgument]:
        """Generate launch arguments for registered parameters.
        
        Returns:
            List of DeclareLaunchArgument objects for use in launch descriptions
        """
        launch_args = []
        for name, param in self.parameters.items():
            # Skip remap parameter as it's handled separately
            if name == "remap":
                continue
                
            if param.launch_arg:  # Only create launch args when flag is True
                value = getattr(self, f"_{name}")
                # For optional parameters with no value, skip creating launch argument
                if not param.required and value is None:
                    continue
                    
                # Convert to string for launch argument
                if value is not None:
                    value = str(value)
                launch_args.append(
                    DeclareLaunchArgument(
                        name,
                        default_value=value if value is not None else "",
                        description=param.help
                    )
                )
        return launch_args

    def get_launch_configuration(self, param_name: str) -> LaunchConfiguration:
        """Get LaunchConfiguration for a parameter.
        
        Args:
            param_name: Name of the parameter
            
        Returns:
            LaunchConfiguration object for use in launch descriptions
            
        Raises:
            ValueError: If parameter is not registered or not a launch argument
        """
        if param_name not in self.parameters:
            raise ValueError(f"Parameter '{param_name}' not registered")
        if not self.parameters[param_name].launch_arg:
            raise ValueError(f"Parameter '{param_name}' is not a launch argument")
        return LaunchConfiguration(param_name)

    def get_remappings(self) -> List[SetRemap]:
        """Get all topic remappings.
        
        The remapping converts from source topics (as they appear in the bag file)
        to target topics (as expected by the nodes).
        
        Example:
            If a bag file has topic '/base_scan' but the node expects '/scan',
            use remapping '/base_scan;/scan'
        
        Returns:
            List of SetRemap actions for all registered remappings
        """
        return [SetRemap(from_topic, to_topic) for from_topic, to_topic in self.remappings]
