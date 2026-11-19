"""The six guided experiments of the Engineering Lab, in teaching order."""

from .attitude import EXPERIMENT as ATTITUDE
from .base import Experiment, ExperimentError
from .mass_thrust import EXPERIMENT as MASS_THRUST
from .pid_tuning import EXPERIMENT as PID_TUNING
from .sensor_noise import EXPERIMENT as SENSOR_NOISE
from .thrust_takeoff import EXPERIMENT as THRUST_TAKEOFF
from .wind import EXPERIMENT as WIND

EXPERIMENTS: dict[str, Experiment] = {
    e.id: e for e in (THRUST_TAKEOFF, MASS_THRUST, ATTITUDE, PID_TUNING, WIND, SENSOR_NOISE)
}


def get_experiment(experiment_id: str) -> Experiment:
    try:
        return EXPERIMENTS[experiment_id]
    except KeyError:
        raise ExperimentError(f"Unknown experiment '{experiment_id}'.") from None


__all__ = ["EXPERIMENTS", "Experiment", "ExperimentError", "get_experiment"]
