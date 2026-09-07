import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import rclpy
from rcl_interfaces.srv import GetParameters
from lifecycle_msgs.srv import ChangeState
import yaml

parser = argparse.ArgumentParser()
parser.add_argument('repo', type=Path)
parser.add_argument('output', type=Path)
args = parser.parse_args()
os.environ['ROS_DOMAIN_ID'] = '217'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
rclpy.init(args=[])
node = rclpy.create_node('universal_slam_review_probe')
results = {}
try:
    for server, package, costmap, config in [
        ('controller_server', 'nav2_controller', 'local_costmap', 'nav2_controller.yaml'),
        ('planner_server', 'nav2_planner', 'global_costmap', 'nav2_planner.yaml'),
    ]:
        path = args.repo / 'slam_nav_core/config' / config
        log_path = args.output.parent / (server + '_probe.log')
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with log_path.open('w') as log:
            proc = subprocess.Popen(
                ['ros2', 'run', package, server, '--ros-args', '--params-file', str(path)],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                lifecycle = node.create_client(ChangeState, '/' + server + '/change_state')
                if not lifecycle.wait_for_service(timeout_sec=10.0):
                    results[server] = {'error': 'lifecycle unavailable'}
                    continue
                request = ChangeState.Request()
                request.transition.id = 1
                transition = lifecycle.call_async(request)
                rclpy.spin_until_future_complete(node, transition, timeout_sec=15.0)
                client = node.create_client(GetParameters, '/' + costmap + '/' + costmap + '/get_parameters')
                if not client.wait_for_service(timeout_sec=15.0):
                    results[server] = {'error': 'parameter service unavailable', 'exit': proc.poll()}
                    continue
                names = ['global_frame', 'robot_base_frame', 'plugins', 'rolling_window',
                         'width', 'height', 'robot_radius']
                future = client.call_async(GetParameters.Request(names=names))
                rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
                if not future.done():
                    results[server] = {'error': 'parameter request timed out'}
                    continue
                values = []
                for value in future.result().values:
                    fields = {1: 'bool_value', 2: 'integer_value', 3: 'double_value',
                              4: 'string_value', 9: 'string_array_value'}
                    values.append(getattr(value, fields[value.type]) if value.type in fields else None)
                configured = yaml.safe_load(path.read_text())[costmap][costmap]['ros__parameters']
                results[server] = {
                    'actual_costmap_parameters': dict(zip(names, values)),
                    'intended_parameters_in_repository': {key: configured.get(key) for key in names},
                    'process_alive_at_query': proc.poll() is None,
                }
            finally:
                if proc.poll() is None:
                    os.killpg(proc.pid, signal.SIGINT)
                    try:
                        proc.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait()
finally:
    node.destroy_node()
    rclpy.shutdown()
args.output.write_text(json.dumps(results, indent=2), encoding='utf-8')
print(json.dumps(results, indent=2))

