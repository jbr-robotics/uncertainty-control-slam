from cartographer_tuner.exceptions import CartographerTunerException

__all__ = [
    'CartographerConfigException',
    'CartographerDependencyException',
    'ConfigLoadException',
    'ConfigParseException',
    'InvalidParameterException',
    'ConfigFileException',
]


class CartographerConfigException(CartographerTunerException):
    """Base exception for Exceptions related to Cartographer configuration."""
    pass


class ConfigLoadException(CartographerConfigException):
    """Exception raised when a configuration file cannot be loaded."""
    pass


class ConfigParseException(CartographerConfigException):
    """Exception raised when a configuration cannot be parsed."""
    pass


class InvalidParameterException(CartographerConfigException):
    """Exception raised when accessing or setting an invalid parameter."""
    pass


class ConfigFileException(CartographerConfigException):
    """Exception raised for file operation Exceptions when saving/loading configs."""
    pass


