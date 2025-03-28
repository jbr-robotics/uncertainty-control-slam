from abc import ABC, abstractmethod
from typing import Dict, Optional, List
from cartographer_tuner.metrics.metric import Metric
from cartographer_tuner.metrics.calculators.exceptions import MetricNotAvailableException

__all__ = ["BaseMetricCalculator"]

class BaseMetricCalculator(ABC):
    """Abstract base class for all metric calculators.
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
            MetricNotAvailableException: If requested metrics are not available
            MetricCalculationException: If calculation of a specific metric fails
        """
        pass

    METRIC_NAMES = []
    
    @classmethod
    def available_metrics(cls) -> List[str]:
        """Get list of available metrics for this calculator.
        
        Returns:
            List of metric names that can be calculated
        """
        return cls.METRIC_NAMES
    
    def _process_metric_names(self, metrics: Optional[List[str]] = None) -> List[str]:
        if metrics is None:
            metrics = self.available_metrics()
        available = set(self.available_metrics())
        invalid = set(metrics) - available
        if invalid:
            raise MetricNotAvailableException(
                requested_metrics=metrics,
                available_metrics=available
            )
        return metrics
    