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