#!/usr/bin/env python3
"""Flat-ground obstacle branch; not a terrain or negative-obstacle detector.

The ground plane is taken relative to the robot's current odometry height, so vertical
LIO drift does not turn the floor into obstacles (or hide low obstacles).
"""
from collections import deque
import numpy as np


def filter_points(xyz, base_xyz, base_height, self_radius, min_height, max_height):
    """Keep points between min/max height above the floor under the robot, outside the self mask."""
    ground = base_xyz[2] - base_height
    self_distance = np.hypot(xyz[:, 0] - base_xyz[0], xyz[:, 1] - base_xyz[1])
    keep = (np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > ground + min_height) &
            (xyz[:, 2] < ground + max_height) & (self_distance > self_radius))
    return xyz[keep]


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from sensor_msgs.msg import PointCloud2, PointField
    from tf2_ros import Buffer, TransformListener, TransformException

    class Filter(Node):
        def __init__(self):
            super().__init__('obstacle_filter')
            self.base_height=float(self.declare_parameter('base_height',0.).value)
            self.radius=float(self.declare_parameter('self_radius',.55).value)
            self.min_h=float(self.declare_parameter('min_obstacle_height',.15).value)
            self.max_h=float(self.declare_parameter('max_obstacle_height',2.).value)
            self.odom=self.declare_parameter('odom_frame','uslam/odom').value
            self.base=self.declare_parameter('base_frame','uslam/base_link').value
            if (not np.isfinite([self.base_height,self.radius,self.min_h,self.max_h]).all()
                    or self.base_height<=0 or self.radius<=0 or not 0<=self.min_h<self.max_h):
                raise ValueError('invalid obstacle filter bounds')
            self.buffer=Buffer(); self.listener=TransformListener(self.buffer,self)
            self.queue=deque(maxlen=5)
            self.pub=self.create_publisher(PointCloud2,self.declare_parameter('output_topic','/uslam/obstacles').value,5)
            self.sub=self.create_subscription(PointCloud2,'/uslam/lio/cloud_registered',self.queue.append,5)
            self.timer=self.create_timer(.05,self.process)

        def process(self):
            if not self.queue: return
            msg=self.queue[-1]
            if msg.header.frame_id!='camera_init': return
            try:
                tf=self.buffer.lookup_transform(self.odom,self.base,Time.from_msg(msg.header.stamp))
            except TransformException:
                return
            self.queue.clear()
            fields={f.name:f for f in msg.fields}
            if not all(k in fields and fields[k].datatype==7 for k in ('x','y','z')): return
            dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,
                'offsets':[fields[k].offset for k in ('x','y','z')],'itemsize':msg.point_step})
            if msg.is_bigendian or len(msg.data)!=msg.row_step*msg.height: return
            src=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step))
            xyz=np.column_stack([src[k].ravel() for k in ('x','y','z')])
            t=tf.transform.translation
            # Circular self mask is yaw-independent; radius must cover legs and payload.
            xyz=np.asarray(filter_points(xyz,(t.x,t.y,t.z),self.base_height,self.radius,self.min_h,self.max_h),dtype='<f4')
            out=PointCloud2(); out.header=msg.header; out.header.frame_id=self.odom
            out.height=1; out.width=len(xyz); out.point_step=12; out.row_step=12*out.width
            out.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(('x','y','z'))]
            out.is_dense=True; out.data=xyz.tobytes(); self.pub.publish(out)

    rclpy.init(); node=Filter()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.try_shutdown()


if __name__=='__main__': main()
