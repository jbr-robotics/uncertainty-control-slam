from cartographer_tuner.exceptions import CartographerTunerException

__all__ = [
    "ExternalToolException", 
    "ExternalToolParameterException",
    "ExternalToolRunException"
]

class ExternalToolException(CartographerTunerException):
    """Base exception for external tool errors."""
    pass

class ExternalToolParameterException(ExternalToolException, ValueError):
    """Exception for invalid external tool parameters."""
    pass

class ExternalToolRunException(ExternalToolException, RuntimeError):
    """Exception for failed external tool runs."""
    pass
