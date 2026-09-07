#!/usr/bin/env python3
"""Flat-ground obstacle branch; not a terrain or negative-obstacle detector."""
import struct
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener, TransformException


class Filter(Node):
    def __init__(self):
        super().__init__('obstacle_filter')
        self.ground=float(self.declare_parameter('ground_z',0.).value)
        self.radius=float(self.declare_parameter('self_radius',.55).value)
        self.odom=self.declare_parameter('odom_frame','uslam/odom').value
        self.base=self.declare_parameter('base_frame','uslam/base_link').value
        if not np.isfinite(self.ground) or not np.isfinite(self.radius) or self.radius<=0:
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
        self_distance=np.hypot(xyz[:,0]-t.x,xyz[:,1]-t.y)
        keep=np.isfinite(xyz).all(axis=1) & (xyz[:,2]>self.ground+.15) & (xyz[:,2]<self.ground+2.) & (self_distance>self.radius)
        xyz=np.asarray(xyz[keep],dtype='<f4')
        out=PointCloud2(); out.header=msg.header; out.header.frame_id=self.odom
        out.height=1; out.width=len(xyz); out.point_step=12; out.row_step=12*out.width
        out.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(('x','y','z'))]
        out.is_dense=True; out.data=xyz.tobytes(); self.pub.publish(out)


def main():
    rclpy.init(); node=Filter()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.try_shutdown()


if __name__=='__main__': main()
