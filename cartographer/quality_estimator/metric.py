from dataclasses import dataclass

@dataclass
class Metric:
    """Container for metric calculation results."""
    name: str
    value: float
    uncertainty: float
    unit: str = ""