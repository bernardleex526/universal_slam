import importlib.util
from pathlib import Path
import numpy as np

spec=importlib.util.spec_from_file_location('odom_adapter',Path(__file__).parents[1]/'scripts/odometry_adapter.py')
mod=importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

def test_rotating_offset_imu_has_stationary_base():
    p,q,v,w=mod.convert_state([1,0,0],[0,0,0,1],[0,2,0],[0,0,2],[1,0,0],[0,0,0,1])
    np.testing.assert_allclose(p,[0,0,0],atol=1e-9)
    np.testing.assert_allclose(v,[0,0,0],atol=1e-9)
    np.testing.assert_allclose(w,[0,0,2],atol=1e-9)

def test_rotated_imu_velocity_is_expressed_in_base():
    s=2**-.5
    p,q,v,w=mod.convert_state([0,0,0],[0,0,s,s],[1,0,0],[0,0,1],[0,0,0],[0,0,s,s])
    np.testing.assert_allclose(v,[0,1,0],atol=1e-9)
    np.testing.assert_allclose(q,[0,0,0,1],atol=1e-9)

