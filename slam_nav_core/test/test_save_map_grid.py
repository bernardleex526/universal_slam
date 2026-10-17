import importlib.util
from pathlib import Path

import numpy as np

spec = importlib.util.spec_from_file_location('save_map', Path(__file__).parents[1] / 'scripts/save_map.py')
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def _floor_with_box(drift_per_m):
    xs, ys = np.meshgrid(np.arange(0, 20, .05), np.arange(0, 6, .05))
    floor = np.c_[xs.ravel(), ys.ravel(), drift_per_m * xs.ravel()]
    bx, by = np.meshgrid(np.arange(15, 15.5, .05), np.arange(3, 3.5, .05))
    box = np.c_[bx.ravel(), by.ravel(), drift_per_m * bx.ravel() + 0.5]
    return np.r_[floor, box]


def test_grid_marks_only_the_box_under_vertical_drift():
    for drift in (0.0, 0.02, -0.02):  # up to 0.4 m over 20 m
        grid, origin = mod.occupancy_grid(_floor_with_box(drift))
        occupied = np.argwhere(grid == 100)
        x = occupied[:, 1] * 0.05 + origin[0]
        y = occupied[:, 0] * 0.05 + origin[1]
        assert len(occupied) == 90
        assert x.min() >= 14.99 and x.max() <= 15.5 and y.min() >= 2.99 and y.max() <= 3.5


def test_pcd_roundtrip_and_map_files(tmp_path):
    xyz = _floor_with_box(0.0).astype(np.float32)
    pcd = tmp_path / 'm.pcd'
    with open(pcd, 'wb') as f:
        f.write(('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n'
                 f'COUNT 1 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                 f'POINTS {len(xyz)}\nDATA binary\n').encode())
        f.write(np.c_[xyz, np.zeros(len(xyz), np.float32)].tobytes())
    np.testing.assert_allclose(mod.read_pcd_xyz(pcd), xyz, atol=1e-6)
    grid, origin = mod.occupancy_grid(mod.read_pcd_xyz(pcd))
    mod.write_map(grid, origin, 0.05, tmp_path / 'grid')
    header = (tmp_path / 'grid.pgm').read_bytes()[:20].split(b'\n')
    assert header[0] == b'P5' and header[1] == b'%d %d' % (grid.shape[1], grid.shape[0])
    assert 'mode: trinary' in (tmp_path / 'grid.yaml').read_text()
