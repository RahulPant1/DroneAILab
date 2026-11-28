"""The guided experiments of the Engineering Lab, in teaching order."""

from .agile_flight import EXPERIMENT as AGILE
from .attitude import EXPERIMENT as ATTITUDE
from .base import Experiment, ExperimentError
from .crop_health import EXPERIMENT as CROP_HEALTH
from .estimators import EXPERIMENT as ESTIMATORS
from .exploration import EXPERIMENT as EXPLORATION
from .mass_thrust import EXPERIMENT as MASS_THRUST
from .missions import EXPERIMENT as MISSIONS
from .pid_tuning import EXPERIMENT as PID_TUNING
from .precision_landing import EXPERIMENT as PRECISION_LANDING
from .sensor_noise import EXPERIMENT as SENSOR_NOISE
from .survey_mapping import EXPERIMENT as SURVEY
from .thrust_takeoff import EXPERIMENT as THRUST_TAKEOFF
from .wind import EXPERIMENT as WIND

EXPERIMENTS: dict[str, Experiment] = {
    e.id: e for e in (THRUST_TAKEOFF, MASS_THRUST, ATTITUDE, PID_TUNING, WIND, SENSOR_NOISE, MISSIONS, SURVEY, PRECISION_LANDING, CROP_HEALTH,
                 AGILE, ESTIMATORS, EXPLORATION)
}


def get_experiment(experiment_id: str) -> Experiment:
    try:
        return EXPERIMENTS[experiment_id]
    except KeyError:
        raise ExperimentError(f"Unknown experiment '{experiment_id}'.") from None


__all__ = ["EXPERIMENTS", "Experiment", "ExperimentError", "get_experiment"]
