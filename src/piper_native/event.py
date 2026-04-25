from dataclasses import dataclass

import numpy as np


@dataclass
class PiperMove:
    hand: str
    prev_rot: tuple[float, float, float, float]
    rot: tuple[float, float, float, float]
    prev_pos: tuple[float, float, float]
    pos: tuple[float, float, float]


@dataclass
class PiperGrip:
    hand: str
    open: bool


@dataclass
class PiperReposition:
    hand: str
    joints: tuple[float, float, float, float, float, float]


@dataclass
class PiperReset:
    hand: str


PiperEvent = PiperMove | PiperGrip | PiperReposition | PiperReset
