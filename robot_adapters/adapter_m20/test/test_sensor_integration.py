import os
os.environ['ROS_DOMAIN_ID'] = '219'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
import signal
import struct
import subprocess
import time
from pathlib import Path
import rclpy
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.qos import qos_profile_sensor_data


def test_robosense_absolute_seconds_are_sorted_relative_times():
    root = Path(__file__).parents[3]
    rclpy.init()
    node = rclpy.create_node('sensor_probe')
    got = []
    node.create_subscription(PointCloud2, '/points_raw', got.append, qos_profile_sensor_data)
    pub = node.create_publisher(PointCloud2, '/LIDAR/POINTS', qos_profile_sensor_data)
    proc = subprocess.Popen([str(root/'build/adapter_m20/m20_topic_bridge')], start_new_session=True,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        deadline = time.monotonic()+5
        while pub.get_subscription_count()==0 and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        msg = PointCloud2()
        msg.header.frame_id='lidar_link'
        msg.header.stamp.sec=1000
        msg.height=1; msg.width=3; msg.point_step=26; msg.row_step=78
        msg.fields=[PointField(name=n,offset=o,datatype=d,count=1) for n,o,d in
                    [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('ring',16,4),('timestamp',18,8)]]
        msg.data=b''.join(struct.pack('<ffffHd',x,0.,0.,1.,ring,t) for x,ring,t in
                          [(3.,191,1000.2),(1.,0,1000.),(2.,100,1000.1)])
        pub.publish(msg)
        deadline=time.monotonic()+5
        while not got and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        assert got, 'no normalized cloud'
        out=got[0]
        fields={f.name:f for f in out.fields}
        assert 'time' in fields, 'FAST-LIO requires relative float time, not absolute timestamp'
        assert fields['time'].datatype==PointField.FLOAT32
        offsets=[struct.unpack_from('<f',out.data,i*out.point_step+fields['time'].offset)[0] for i in range(3)]
        assert abs(offsets[0])<1e-6 and abs(offsets[2]-.2)<1e-5
        assert offsets==sorted(offsets)
        assert out.header.stamp.sec==1000
    finally:
        os.killpg(proc.pid,signal.SIGINT); proc.wait(timeout=5)
        node.destroy_node(); rclpy.shutdown()

