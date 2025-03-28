from typing import Any, Dict, Type
from dataclasses import dataclass, field

__all__ = [
    "ToolParameter"
]

@dataclass
class ToolParameter:
    type: Type
    required: bool = True
    help: str = ""
    default: Any = None
    argparse_kwargs: Dict[str, Any] = field(default_factory=dict)
