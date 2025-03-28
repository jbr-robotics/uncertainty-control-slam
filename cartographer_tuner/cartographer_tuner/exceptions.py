"""Module defines base class for all exceptions raised within cartographer_tuner package."""

__all__ = [
    'CartographerTunerError',
    'CartographerDependencyError',
]


class CartographerTunerError(Exception):
    """Base for cartographer_tuner's exceptions."""

    pass

class CartographerDependencyError(CartographerTunerError):
    """Exception raised when a required external dependency is missing."""
    pass