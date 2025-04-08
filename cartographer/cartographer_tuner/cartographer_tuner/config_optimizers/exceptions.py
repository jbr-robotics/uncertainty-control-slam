from cartographer_tuner.exceptions import CartographerTunerException

class OptimizerException(CartographerTunerException):
    """Base exception for optimizer errors."""
    pass

class OptimizerInvalidArgumentException(OptimizerException, ValueError):
    """Exception raised when optimizer arguments are invalid."""
    pass

