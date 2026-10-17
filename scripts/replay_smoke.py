#!/usr/bin/env python3
"""Replay only sensor topics in an isolated localhost domain; never enable control."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np

parser=argparse.ArgumentParser()
parser.add_argument('bag',type=Path)
parser.add_argument('output',type=Path)
parser.add_argument('--rate',type=float,default=.5)
parser.add_argument('--verify-grid',action='store_true')
args=parser.parse_args()
if args.rate<=0: parser.error('rate must be positive')
args.output=args.output.resolve(); args.output.mkdir(parents=True,exist_ok=True)
os.environ.update(ROS_DOMAIN_ID='221',ROS_LOCALHOST_ONLY='1')
os.environ.setdefault('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp')
from ament_index_python.packages import get_package_share_directory
if os.environ['RMW_IMPLEMENTATION']=='rmw_cyclonedds_cpp':
    os.environ.setdefault('CYCLONEDDS_URI','file://'+os.path.join(
        get_package_share_directory('slam_nav_core'),'config','cyclonedds_localhost.xml'))
profile=os.path.join(get_package_share_directory('m20pro_bridge'),'config','m20_official_bag.yaml')
import rclpy
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
rclpy.init()
node=rclpy.create_node('offline_acceptance_probe')
samples=[]; edges=set(); counts={}
from sensor_msgs.msg import PointCloud2, Imu
from rclpy.qos import QoSProfile
probe_qos=QoSProfile(depth=200)
for topic,kind in [('/IMU',Imu),('/LIDAR/POINTS',PointCloud2),('/uslam/imu_raw',Imu),('/uslam/points_raw',PointCloud2)]:
    counts[topic]=0
    def count(msg,topic=topic): counts[topic]+=1
    node.create_subscription(kind,topic,count,probe_qos)
def odom(msg):
    p=msg.pose.pose.position; v=msg.twist.twist.linear; w=msg.twist.twist.angular
    samples.append([msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,p.x,p.y,p.z,v.x,v.y,v.z,w.z])
node.create_subscription(Odometry,'/uslam/odom',odom,100)
node.create_subscription(TFMessage,'/tf',lambda msg:edges.update((t.header.frame_id,t.child_frame_id) for t in msg.transforms),100)
log=(args.output/'launch.log').open('w')
stack=subprocess.Popen(['ros2','launch','universal_slam_bringup','universal_slam.launch.py',
    'robot_config:='+profile,'use_bridge:=true','use_sim_time:=true',
    'enable_control:=false','use_nav:=false','map_file_path:='+str(args.output/'map.pcd')],
    stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
player=None
result={}
try:
    deadline=time.monotonic()+20
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.1)
        if node.count_subscribers('/LIDAR/POINTS') and node.count_subscribers('/uslam/points_raw'):
            break
        if stack.poll() is not None: raise RuntimeError('stack exited; see launch.log')
    else: raise RuntimeError('sensor subscriptions not ready')
    with (args.output/'play.log').open('w') as playback:
        player=subprocess.Popen(['ros2','bag','play',str(args.bag.resolve()),'--clock','100','--rate',str(args.rate),
            '--topics','/IMU','/LIDAR/POINTS'],stdout=playback,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+45.5/args.rate+40
        while player.poll() is None and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if stack.poll() is not None: raise RuntimeError('stack exited during replay')
        if player.poll() is None: raise RuntimeError('replay timeout')
        if player.returncode!=0: raise RuntimeError('bag player failed')
    deadline=time.monotonic()+2
    while time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.1)
    client=node.create_client(Trigger,'/uslam/map_save')
    if not client.wait_for_service(timeout_sec=5): raise RuntimeError('save service missing')
    future=client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node,future,timeout_sec=30)
    saved=future.done() and future.result().success
    if saved:
        command=['python3',str(Path(__file__).parents[1]/'slam_nav_core/scripts/save_map.py'),'map2d','--out-dir',str(args.output)]
        if args.verify_grid: command += ['--convert']
        subprocess.run(command,check=True,timeout=65)
    a=np.asarray(samples)
    result={'topic_counts':counts,'odom_messages':len(samples),'map_save_success':bool(saved),
        'map_bytes':(args.output/'map.pcd').stat().st_size if saved else 0,
        'dynamic_tf_edges':sorted(edges),'control_enabled':False,
        'replay_rate':args.rate,'accuracy_validated':False,'grid_saved':(args.output/'map2d.yaml').is_file()}
    if len(samples):
        result.update(finite=bool(np.isfinite(a).all()),
            max_linear_speed=float(np.linalg.norm(a[:,4:7],axis=1).max()),
            position_min=a[:,1:4].min(axis=0).tolist(),position_max=a[:,1:4].max(axis=0).tolist(),
            final_position=a[-1,1:4].tolist(),monotonic_timestamps=bool((np.diff(a[:,0])>0).all()))
    (args.output/'result.json').write_text(json.dumps(result,indent=2))
    if len(samples): np.savetxt(args.output/'trajectory.csv',a,delimiter=',',header='time,x,y,z,vx,vy,vz,wz')
    print(json.dumps(result,indent=2))
    if not saved or len(samples)<10 or not result.get('finite',False): raise RuntimeError('smoke checks failed')
finally:
    for process in (player,stack):
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try: process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL); process.wait()
    log.close(); node.destroy_node(); rclpy.try_shutdown()
