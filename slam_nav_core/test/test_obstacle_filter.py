import importlib.util
from pathlib import Path
import numpy as np

spec=importlib.util.spec_from_file_location('obstacle_filter',Path(__file__).parents[1]/'scripts/obstacle_filter.py')
mod=importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def test_ground_follows_robot_height_under_vertical_drift():
    floor, box = 0.0, 0.5
    cloud = np.array([[3.0, 0.0, floor], [3.0, 0.0, box], [0.1, 0.0, box]])
    for drift in (0.0, 0.3, -0.3):
        drifted = cloud + [0.0, 0.0, drift]
        kept = mod.filter_points(drifted, (0.0, 0.0, 0.47 + drift), 0.47, 0.55, 0.15, 2.0)
        np.testing.assert_allclose(kept, [[3.0, 0.0, box + drift]])


def test_non_finite_points_are_dropped():
    cloud = np.array([[3.0, 0.0, np.nan], [3.0, 0.0, 0.5]])
    assert len(mod.filter_points(cloud, (0, 0, 0.47), 0.47, 0.55, 0.15, 2.0)) == 1
