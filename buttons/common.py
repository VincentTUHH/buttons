from dataclasses import dataclass
from typing import Any


@dataclass
class SingleData:
    value: Any = None
    stamp: float = 0.0
    updated: bool = False
