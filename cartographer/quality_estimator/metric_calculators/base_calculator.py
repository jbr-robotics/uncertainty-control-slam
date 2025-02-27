from abc import ABC, abstractmethod
from typing import Dict, Optional, List, Type, ClassVar, Set
from quality_estimator.metric import Metric

class BaseMetricCalculator(ABC):
    """Abstract base class for all metric calculators.
    
    This class provides a common interface for different types of metrics:
    - Map quality metrics (MME, MPV, MOM)
    - Relational metrics (from ground truth comparison)
    - Any other metrics that may be added in the future
    
    The interface allows for:
    1. Standardized metric calculation
    2. Consistent result format
    3. Metric availability checking
    """
    
    @abstractmethod
    def calculate(self, metrics: Optional[List[str]] = None) -> Dict[str, Metric]:
        """Calculate metrics.
        
        Args:
            metrics: Optional list of specific metrics to calculate.
                    If None, calculate all available metrics.
        
        Returns:
            Dictionary mapping metric names to their results
            
        Raises:
            ValueError: If calculation fails due to invalid input or unavailable metrics
            RuntimeError: If calculation fails due to execution error
        """
        pass
    
    @staticmethod
    @abstractmethod
    def get_available_metrics() -> List[str]:
        """Get list of available metrics for this calculator.
        
        Returns:
            List of metric names that can be calculated
        """
        pass
    
    def validate_metrics(self, requested_metrics: List[str]) -> None:
        """Validate that requested metrics are available.
        
        Args:
            requested_metrics: List of metric names to validate
            
        Raises:
            ValueError: If any requested metric is not available
        """
        available = set(self.get_available_metrics())
        invalid = set(requested_metrics) - available
        if invalid:
            raise ValueError(
                f"Requested metrics {invalid} are not available. "
                f"Available metrics: {available}"
            )
