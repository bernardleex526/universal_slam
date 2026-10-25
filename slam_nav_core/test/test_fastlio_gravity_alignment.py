"""Tilted start: FAST-LIO keeps its upstream camera_init (start IMU attitude), publishes its
gravity estimate once, and the odometry adapter turns that into a gravity-aligned odom.

Without the alignment a 3 deg pitch at power-on tilts the whole odom frame and the flat-ground
obstacle cut turns the floor into obstacles from ~2.4 m on.
"""
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).parents[2]
PITCH, ROLL = math.radians(6.0), math.radians(3.0)


def _r_ws():
    cp, sp, cr, sr = math.cos(PITCH), math.sin(PITCH), math.cos(ROLL), math.sin(ROLL)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return ry @ rx


def _room_scan(r_ws, lines=32, cols=360, half_fov_deg=30.0):
    """Static tilted sensor inside an ~8 x 6 x 3 m box; points in the sensor frame.

    Walls are off the 0.5 m voxel grid and ranges carry 1 cm noise like a real scan; noise-free
    planes lying exactly on voxel boundaries are a degenerate input for the ikd-tree map.
    """
    elev = np.radians(np.linspace(-half_fov_deg, half_fov_deg, lines))
    az = np.radians(np.arange(cols) * 360.0 / cols)
    d = np.stack([np.cos(elev)[None] * np.cos(az)[:, None], np.cos(elev)[None] * np.sin(az)[:, None],
                  np.broadcast_to(np.sin(elev)[None], (cols, lines))], -1).reshape(-1, 3)
    dw = d @ r_ws.T
    hi, lo = np.array([4.13, 3.07, 2.43]), np.array([-3.91, -2.94, -0.53])
    with np.errstate(divide='ignore'):
        t = np.min(np.where(dw > 0, hi / dw, np.where(dw < 0, lo / dw, np.inf)), axis=1)
    t = t + np.random.default_rng(7).normal(0.0, 0.01, t.shape)
    ring = np.tile(np.arange(lines, dtype=np.uint16), cols)
    offset = np.repeat(np.arange(cols) * 0.1 / cols, lines).astype(np.float32)
    return d * t[:, None], ring, offset


def _rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def test_tilted_start_yields_gravity_aligned_odom(tmp_path):
    os.environ.update(ROS_DOMAIN_ID='216', ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                      CYCLONEDDS_URI='file://' + str(ROOT / 'slam_nav_core/config/cyclonedds_localhost.xml'))
    import rclpy
    from geometry_msgs.msg import Vector3Stamped
    from nav_msgs.msg import Odometry
    from rclpy.duration import Duration
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu, PointCloud2, PointField
    proc = subprocess.Popen([str(ROOT / 'build/fast_lio/fastlio_mapping'), '--ros-args',
                             '--params-file', str(ROOT / 'slam_nav_core/config/fastlio_params.yaml'),
                             '-p', 'common.lid_topic:=/gtest/points', '-p', 'common.imu_topic:=/gtest/imu',
                             '-p', 'preprocess.scan_line:=32', '-p', 'preprocess.blind:=0.3',
                             '-p', 'mapping.extrinsic_T:=[0.0,0.0,0.0]',
                             '-p', 'mapping.extrinsic_R:=[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]',
                             '-p', 'map_file_path:=' + str(tmp_path / 'map.pcd')],
                            start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    adapter = subprocess.Popen(['python3', str(ROOT / 'slam_nav_core/scripts/odometry_adapter.py'), '--ros-args',
                                '-p', 'input_topic:=/Odometry', '-p', 'gravity_topic:=/gravity_init',
                                '-p', 'output_topic:=/gtest/odom', '-p', 'odom_frame:=gtest/odom',
                                '-p', 'base_frame:=gtest/base', '-p', 'lio_frame:=gtest/lio'],
                               start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    node = rclpy.create_node('gravity_probe')
    reliable = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
    imu_pub = node.create_publisher(Imu, '/gtest/imu', reliable)
    cloud_pub = node.create_publisher(PointCloud2, '/gtest/points', QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
    odoms, lio, gravity = [], [], []
    node.create_subscription(Odometry, '/gtest/odom', odoms.append, 10)
    node.create_subscription(Odometry, '/Odometry', lio.append, 10)
    node.create_subscription(Vector3Stamped, '/gravity_init', gravity.append,
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    r_ws = _r_ws()
    f_s = r_ws.T @ np.array([0.0, 0.0, 9.81])  # static specific force in the sensor frame
    xyz, ring, offset = _room_scan(r_ws)
    dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'), ('time', '<f4'), ('ring', '<u2')])
    cloud = np.empty(len(xyz), dtype)
    cloud['x'], cloud['y'], cloud['z'] = xyz.T
    cloud['intensity'], cloud['time'], cloud['ring'] = 10.0, offset, ring
    fields = [PointField(name=n, offset=o, datatype=d, count=1) for n, o, d in
              [('x', 0, 7), ('y', 4, 7), ('z', 8, 7), ('intensity', 12, 7), ('time', 16, 7), ('ring', 20, 4)]]
    try:
        deadline = time.monotonic() + 25
        next_cloud = time.monotonic() + 0.5
        while time.monotonic() < deadline and len(odoms) < 40:
            now = node.get_clock().now()
            m = Imu()
            m.header.stamp, m.header.frame_id = now.to_msg(), 'imu'
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = map(float, f_s)
            m.orientation.w = 1.0
            imu_pub.publish(m)
            if time.monotonic() >= next_cloud:
                next_cloud += 0.1
                c = PointCloud2()
                c.header.stamp = (now - Duration(seconds=0.1)).to_msg()
                c.header.frame_id = 'lidar'
                c.height, c.width, c.point_step, c.row_step = 1, len(cloud), dtype.itemsize, dtype.itemsize * len(cloud)
                c.fields, c.is_dense, c.data = fields, True, cloud.tobytes()
                cloud_pub.publish(c)
            rclpy.spin_once(node, timeout_sec=0.005)
        assert len(odoms) >= 40, 'adapter produced too little odometry'
        assert gravity and gravity[-1].header.frame_id == 'camera_init', 'FAST-LIO must publish its gravity'
        f_hat = f_s / np.linalg.norm(f_s)
        # Upstream FAST-LIO numerics are kept: camera_init stays at the start attitude.
        lio_tilt = math.degrees(math.acos(min(1.0, (_rot(lio[-1].pose.pose.orientation) @ f_hat)[2])))
        assert lio_tilt > 5.5, f'camera_init should keep the start attitude, tilt {lio_tilt:.2f} deg'
        r_ob = _rot(odoms[-1].pose.pose.orientation)
        tilt = math.degrees(math.acos(min(1.0, (r_ob @ f_hat)[2])))
        assert tilt < 0.75, f'odom is tilted {tilt:.2f} deg from gravity (start tilt was 6.7 deg)'
        heading = math.degrees(math.atan2(r_ob[1, 0], r_ob[0, 0]))
        assert abs(heading) < 1.0, f'odom x must follow the base start heading, got {heading:.2f} deg'
        p = odoms[-1].pose.pose.position
        assert math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z) < 0.05
    finally:
        for child in (adapter, proc):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
            child.wait(timeout=10)
        node.destroy_node()
        rclpy.shutdown()
