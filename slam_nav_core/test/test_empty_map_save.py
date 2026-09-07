import os
import signal
import subprocess
from pathlib import Path
import rclpy
from std_srvs.srv import Trigger


def test_empty_map_fails_without_crashing(tmp_path):
    os.environ['ROS_DOMAIN_ID']='220'
    os.environ['ROS_LOCALHOST_ONLY']='1'
    os.environ['RMW_IMPLEMENTATION']='rmw_cyclonedds_cpp'
    root=Path(__file__).parents[2]
    proc=subprocess.Popen([str(root/'build/fast_lio/fastlio_mapping'),'--ros-args',
        '--params-file',str(root/'slam_nav_core/config/fastlio_params.yaml'),
        '-p','map_file_path:='+str(tmp_path/'map.pcd')],start_new_session=True,
        stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    rclpy.init()
    node=rclpy.create_node('save_probe')
    try:
        client=node.create_client(Trigger,'/map_save')
        assert client.wait_for_service(timeout_sec=5)
        future=client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        assert future.done(), 'empty save must return failure instead of aborting'
        assert not future.result().success
        assert proc.poll() is None
        assert not (tmp_path/'map.pcd').exists()
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
        proc.wait(timeout=5)
        node.destroy_node(); rclpy.shutdown()

