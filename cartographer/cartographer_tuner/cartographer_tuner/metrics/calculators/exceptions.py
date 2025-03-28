"""Exceptions for metric calculation operations."""

from typing import List, Set, Optional
from cartographer_tuner.exceptions import CartographerTunerException


class MetricCalculatorException(CartographerTunerException):
    """Base exception for all metric-related Exceptions."""
    pass


class MetricNotAvailableException(MetricCalculatorException):
    """Exception raised when requested metrics are not available."""
    
    def __init__(
        self, 
        requested_metrics: List[str], 
        available_metrics: Set[str],
        message: Optional[str] = None
    ):
        self.requested_metrics = requested_metrics
        self.available_metrics = available_metrics
        self.invalid_metrics = set(requested_metrics) - available_metrics
        
        if message is None:
            message = (
                f"Requested metrics {self.invalid_metrics} are not available. "
                f"Available metrics: {self.available_metrics}"
            )
        
        super().__init__(message)


class MetricCalculationException(MetricCalculatorException):
    """Exception raised when metric calculation fails."""
    pass

class CalculatorFileNotFoundException(MetricCalculatorException, FileNotFoundError):
    """Exception raised when calculator input file is not found."""
    pass

class CalculatorFileFormatException(MetricCalculatorException, ValueError):
    """Exception raised when calculator input file has invalid format."""
    pass


