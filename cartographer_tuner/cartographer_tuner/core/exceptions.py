from cartographer_tuner.exceptions import CartographerTunerError

__all__ = [
    'CartographerConfigError',
    'CartographerDependencyError',
    'ConfigLoadError',
    'ConfigParseError',
    'InvalidParameterError',
    'ConfigFileError',
]


class CartographerConfigError(CartographerTunerError):
    """Base exception for errors related to Cartographer configuration."""
    pass


class ConfigLoadError(CartographerConfigError):
    """Exception raised when a configuration file cannot be loaded."""
    pass


class ConfigParseError(CartographerConfigError):
    """Exception raised when a configuration cannot be parsed."""
    pass


class InvalidParameterError(CartographerConfigError):
    """Exception raised when accessing or setting an invalid parameter."""
    pass


class ConfigFileError(CartographerConfigError):
    """Exception raised for file operation errors when saving/loading configs."""
    pass


