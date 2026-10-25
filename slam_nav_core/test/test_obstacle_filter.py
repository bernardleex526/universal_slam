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


def test_far_points_dropped_and_columns_thinned():
    wall = np.array([[3.0, y, z] for y in np.arange(-1, 1, 0.01) for z in (0.3, 0.8, 1.5)])
    far = np.array([[12.0, 0.0, 0.5]])
    kept = mod.filter_points(np.r_[wall, far], (0.0, 0.0, 0.47), 0.47, 0.55, 0.15, 2.0, max_range=9.0)
    assert len(kept) == len(wall)
    thin = mod.thin_to_grid(kept, 0.05)
    cells = {tuple(c) for c in np.floor(kept[:, :2] / 0.05).astype(int)}
    assert len(thin) == len(cells), 'exactly one point per occupied x/y cell'
    assert {tuple(c) for c in np.floor(thin[:, :2] / 0.05).astype(int)} == cells
    assert len(mod.thin_to_grid(np.empty((0, 3)), 0.05)) == 0
