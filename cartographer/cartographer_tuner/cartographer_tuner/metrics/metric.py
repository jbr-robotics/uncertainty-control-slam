from dataclasses import dataclass
from typing import Optional

__all__ = ["Metric"]

@dataclass
class Metric:
    """Container for metric calculation results."""
    name: str
    value: float
    uncertainty: Optional[float] = None
    unit: Optional[str] = None

    def __str__(self) -> str:
        if self.uncertainty is not None:
            value_str = f"{self.value:.5f} +- {self.uncertainty:.5f}"
        else:
            value_str = f"{self.value:.5f}"
            
        if self.unit:
            return f"{self.name}: {value_str} {self.unit}"
        else:
            return f"{self.name}: {value_str}"