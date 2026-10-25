"""Back-end node contract: fails fast without a map, refuses to save an empty session, strict relocalization."""
import os
import signal
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).parents[2]
EXE = ROOT / 'build/uslam_backend/backend_node'


def _env():
    return dict(os.environ, ROS_DOMAIN_ID='215', ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                CYCLONEDDS_URI='file://' + str(ROOT / 'slam_nav_core/config/cyclonedds_localhost.xml'))


def test_localization_without_saved_map_fails_fast(tmp_path):
    proc = subprocess.run([str(EXE), '--ros-args', '-p', 'mode:=localization', '-p', 'map_dir:=' + str(tmp_path / 'none')],
                          env=_env(), capture_output=True, text=True, timeout=20)
    assert proc.returncode != 0
    assert 'saved map' in proc.stderr + proc.stdout


def test_mapping_save_refuses_empty_session(tmp_path):
    os.environ.update(_env())
    import rclpy
    from std_srvs.srv import Trigger
    proc = subprocess.Popen([str(EXE), '--ros-args', '-p', 'mode:=mapping', '-p', 'map_dir:=' + str(tmp_path / 'map')],
                            env=_env(), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('backend_probe')
    try:
        client = node.create_client(Trigger, '/uslam_backend/save_map')
        assert client.wait_for_service(timeout_sec=10)
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        assert future.done() and not future.result().success
        assert not (tmp_path / 'map').exists()
        time.sleep(0.5)
        assert proc.poll() is None
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=10)
        node.destroy_node()
        rclpy.shutdown()


def test_relocalization_accepts_only_near_complete_explanations():
    """Closed-loop simulation: correct global fixes scored inlier ratio 1.00, wrong ones in similar rooms
    (unmapped neighbour room after LOST, point-symmetric copy from the map-origin guess) 0.67-0.69."""
    import yaml
    params = next(iter(yaml.safe_load((ROOT / 'uslam_backend/config/backend.yaml').read_text()).values()))['ros__parameters']
    assert params['reloc_min_inlier_ratio'] >= 0.8
    assert params['reloc_min_inlier_ratio'] > params['track_min_inlier_ratio'], 'global search is stricter than tracking'
    assert params['reloc_ambiguity_margin'] > 0
    # enough spatially distinct candidates that a symmetric twin cannot crowd out the true place
    assert params['relocalization_candidates'] >= 10 and params['relocalization_candidate_spacing_m'] > 0
