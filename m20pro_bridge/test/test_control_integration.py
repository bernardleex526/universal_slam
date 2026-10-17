import os
from pathlib import Path
import signal
import socket
import subprocess


def test_direct_node_start_does_not_send_robot_commands():
    root = Path(__file__).parents[2]
    exe = root / 'build/m20pro_bridge/m20_cmd_vel_bridge'
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as peer:
        peer.bind(('127.0.0.1', 0))
        peer.settimeout(1.5)
        env = dict(os.environ, ROS_DOMAIN_ID='218', ROS_LOCALHOST_ONLY='1',
                   RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
        proc = subprocess.Popen([str(exe), '--ros-args', '-p', 'robot_ip:=127.0.0.1',
                                '-p', 'port:=' + str(peer.getsockname()[1])],
                               env=env, stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL, start_new_session=True)
        try:
            try:
                packet = peer.recv(65535)
            except socket.timeout:
                packet = b''
            assert not packet, 'starting a node must not send mode/stand/velocity commands'
            assert proc.poll() is None
        finally:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=5)


def test_arming_limits_and_feedback_timeout():
    import json
    import struct
    import time
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_srvs.srv import SetBool
    os.environ.update(ROS_DOMAIN_ID='218',ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
    rclpy.init()
    node=rclpy.create_node('control_acceptance')
    odom_pub=node.create_publisher(Odometry,'/odom',5)
    cmd_pub=node.create_publisher(Twist,'/cmd_vel',1)
    root=Path(__file__).parents[2]
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as peer:
        peer.bind(('127.0.0.1',0)); peer.setblocking(False)
        params=['expected_gait:=12290','enable_control:=true','calibration_confirmed:=true','robot_ip:=127.0.0.1',
                'port:='+str(peer.getsockname()[1]),'vx_full_scale:=2.0','vy_full_scale:=2.0',
                'wz_full_scale:=2.0','status_timeout_s:=0.3','heartbeat_hz:=5.0']
        proc=subprocess.Popen([str(root/'build/m20pro_bridge/m20_cmd_vel_bridge'),'--ros-args']+
            [item for p in params for item in ('-p',p)],start_new_session=True,
            stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        address=None; axes=[]
        def pump(duration,feedback=True,commands=True):
            nonlocal address
            end=time.monotonic()+duration
            while time.monotonic()<end:
                odom=Odometry(); odom.header.stamp=node.get_clock().now().to_msg()
                odom.header.frame_id="odom"; odom.child_frame_id="base_link"
                odom.pose.pose.orientation.w=1.; odom_pub.publish(odom)
                if commands:
                    cmd=Twist(); cmd.linear.x=4.; cmd_pub.publish(cmd)
                try:
                    while True:
                        packet,address=peer.recvfrom(65535)
                        body=json.loads(packet[16:])['PatrolDevice']
                        if body['Type']==2 and body['Command']==21:
                            axes.append(body['Items']['X'])
                except BlockingIOError: pass
                if feedback and address:
                    body=json.dumps({'PatrolDevice':{'Type':1002,'Command':6,'Items':{
                        'BasicStatus':{'MotionState':17,'ControlUsageMode':0,'HES':0,'Direction':0,'Charge':0,'Gait':12290}}}}).encode()
                    header=bytes.fromhex('eb91eb90')+struct.pack('<HHB',len(body),42,1)+b'\0'*7
                    peer.sendto(header+body,address)
                rclpy.spin_once(node,timeout_sec=.02)
        try:
            client=node.create_client(SetBool,'/m20_cmd_vel_bridge/arm')
            assert client.wait_for_service(timeout_sec=5)
            future=client.call_async(SetBool.Request(data=True))
            rclpy.spin_until_future_complete(node,future,timeout_sec=2)
            assert future.done() and not future.result().success
            pump(.5)
            future=client.call_async(SetBool.Request(data=True))
            rclpy.spin_until_future_complete(node,future,timeout_sec=2)
            assert future.done() and future.result().success
            pump(.2)
            assert any(abs(x-.1)<1e-8 for x in axes), '4m/s request must clamp to .2m/s / 2m/s full-scale'
            assert max(axes)<=.1+1e-8
            pump(.6,feedback=False)
            assert axes[-1]==0., 'loss of feedback must stop even with fresh cmd_vel'
            axes.clear(); pump(.4,feedback=False)
            assert len(axes)>=4 and all(x==0 for x in axes), 'halt must keep sending zero until explicit handoff/rearm'
            axes.clear(); pump(.4,feedback=True)
            assert not any(x!=0 for x in axes), 'feedback recovery must not automatically re-arm'
        finally:
            if proc.poll() is None: os.killpg(proc.pid,signal.SIGINT)
            proc.wait(timeout=5)
            node.destroy_node(); rclpy.shutdown()
