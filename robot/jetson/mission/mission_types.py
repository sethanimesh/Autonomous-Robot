"""Dependency-free message types shared by mission components."""

from dataclasses import dataclass


@dataclass(frozen=True)
class TargetObservation:
    confirmed: bool
    age_seconds: float
    box_height_fraction: float = 0.0
