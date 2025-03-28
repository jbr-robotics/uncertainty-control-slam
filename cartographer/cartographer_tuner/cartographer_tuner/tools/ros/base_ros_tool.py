from abc import ABC, abstractmethod
from typing import List
import rclpy
from launch import LaunchService, LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from cartographer_tuner.tools.exceptions import ExternalToolParameterException
from cartographer_tuner.tools.base_tool import BaseTool

__all__ = [
    "BaseRosTool"
]

class BaseRosTool(BaseTool):
    """
    Abstract base class for all ros tools.
    """

    @abstractmethod
    def generate_launch_description(self) -> LaunchDescription:
        pass

    def run(self) -> None:
        rclpy.init()
        
        try:
            launch_service = LaunchService()
            launch_service.include_launch_description(self.generate_launch_description())
            launch_service.run()
        finally:
            rclpy.shutdown()

    def get_launch_arguments(self) -> List[DeclareLaunchArgument]:
        launch_args = []
        for name, param in self.parameters.items():
            value = getattr(self, f"_{name}")

            if not param.required and value is None:
                continue
                
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
        if param_name not in self.parameters:
            raise ExternalToolParameterException(f"Parameter '{param_name}' not registered")
        return LaunchConfiguration(param_name)

