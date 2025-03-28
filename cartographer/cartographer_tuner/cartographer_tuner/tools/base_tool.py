from abc import ABC, abstractmethod
import argparse
from typing import ClassVar, Dict, Any, Type

from cartographer_tuner.tools.tool_parameter import ToolParameter

class BaseTool(ABC):
    """Abstract base class for all external tools."""

    parameters: ClassVar[Dict[str, ToolParameter]] = {}

    @abstractmethod
    def run(self) -> None:
        pass

    @classmethod
    @abstractmethod
    def _register_params(cls):
        pass

    def __init__(self, **kwargs):
        for name, param in self.parameters.items():
            if param.required and name not in kwargs:
                raise ValueError(f"Required parameter '{name}' not provided")
            
            value = kwargs.get(name, param.default)
            setattr(self, f"_{name}", value)

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        cls.parameters = {}
        cls._register_params()

    @classmethod
    def register_parameter(
        cls,
        name: str,
        param_type: Type,
        required: bool = True,
        help: str = "",
        default: Any = None,
        **argparse_kwargs
    ) -> None:
        cls.parameters[name] = ToolParameter(
            type=param_type,
            required=required,
            help=help,
            default=default,
            argparse_kwargs=argparse_kwargs
        )

    @classmethod
    def generate_main(cls) -> callable:
        def main():
            parser = argparse.ArgumentParser(description=cls.__doc__)
            
            for name, param in cls.parameters.items():
                arg_params = {
                    "type": param.type,
                    "required": param.required,
                    "help": param.help,
                    "default": param.default,
                    **param.argparse_kwargs
                }
                parser.add_argument(f"--{name}", **arg_params)
            
            args = parser.parse_args()
            
            tool = cls(**vars(args))
            
            tool.run()
        
        return main
