"""Module defines base class for all exceptions raised within cartographer_tuner package."""

__all__ = [
    'CartographerTunerException',
    'CartographerDependencyException',
]


class CartographerTunerException(Exception):
    """Base for cartographer_tuner's exceptions."""

    pass

class CartographerDependencyException(CartographerTunerException):
    """Exception raised when a required external dependency is missing."""
    pass