#!/usr/bin/env python3
"""Convert IMU-centered LIO odometry to base-centered, gravity-aligned odometry, with a divergence latch.

FAST-LIO keeps its upstream world frame camera_init (the IMU attitude at initialisation, not
gravity-aligned). The odom frame is camera_init rotated once by the gravity FAST-LIO estimated
at initialisation: z up, x along the base start heading, origin at the base start position.
"""
import numpy as np


def rotation(q):
    q=np.asarray(q,dtype=float)
    if q.shape!=(4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-3:
        raise ValueError('invalid quaternion')
    x,y,z,w=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def multiply(a,b):
    av,bv=np.asarray(a[:3]),np.asarray(b[:3])
    return np.r_[a[3]*bv+b[3]*av+np.cross(av,bv),a[3]*b[3]-np.dot(av,bv)]


def skew(v):
    x,y,z=v
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])


def quaternion(r):
    """Rotation matrix -> (x, y, z, w), w >= 0."""
    w=np.sqrt(max(0.,1+r[0,0]+r[1,1]+r[2,2]))/2
    x=np.copysign(np.sqrt(max(0.,1+r[0,0]-r[1,1]-r[2,2]))/2,r[2,1]-r[1,2])
    y=np.copysign(np.sqrt(max(0.,1-r[0,0]+r[1,1]-r[2,2]))/2,r[0,2]-r[2,0])
    z=np.copysign(np.sqrt(max(0.,1-r[0,0]-r[1,1]+r[2,2]))/2,r[1,0]-r[0,1])
    q=np.array([x,y,z,w])
    return q/np.linalg.norm(q)


def odom_from_gravity(gravity,base_imu_xyz,base_imu_quat):
    """T_odom_ci as (R_oc, p_oc): p_odom = R_oc p_ci + p_oc.

    At FAST-LIO initialisation R_ci_I = I and p_ci_I = 0, so the base starts at T_ci_B0 = T_IB.
    odom z = -gravity, x = base start x-axis projected on the horizontal plane, y = z cross x,
    origin = base start position. Also well defined for an upside-down IMU.
    """
    g=np.asarray(gravity,dtype=float)
    if g.shape!=(3,) or not np.isfinite(g).all() or np.linalg.norm(g)<1.:
        raise ValueError('invalid gravity')
    up=-g/np.linalg.norm(g)
    r_bi=rotation(base_imu_quat)
    heading=r_bi.T @ np.array([1.,0.,0.])
    x=heading-(heading@up)*up
    if np.linalg.norm(x)<1e-3:
        raise ValueError('base x-axis is vertical; heading undefined')
    x/=np.linalg.norm(x)
    r_oc=np.column_stack([x,np.cross(up,x),up]).T
    p_ci_b0=-r_bi.T @ np.asarray(base_imu_xyz,dtype=float)
    return r_oc,-r_oc @ p_ci_b0


def apply_alignment(r_oc,p_oc,position,quat):
    """Base pose camera_init -> odom."""
    return r_oc @ np.asarray(position,dtype=float)+p_oc,quaternion(r_oc @ rotation(quat))


def convert_state(position,quaternion,velocity,angular,base_imu_xyz,base_imu_quat):
    r_wi,r_bi=rotation(quaternion),rotation(base_imu_quat)
    t_ib=-r_bi.T @ np.asarray(base_imu_xyz)
    p=np.asarray(position)+r_wi @ t_ib
    q=multiply(quaternion,np.r_[-np.asarray(base_imu_quat[:3]),base_imu_quat[3]])
    v=r_bi @ (np.asarray(velocity)+np.cross(angular,t_ib))
    w=r_bi @ np.asarray(angular)
    if not np.isfinite(np.r_[p,q,v,w]).all():
        raise ValueError('non-finite odometry')
    return p,q,v,w


class DivergenceGuard:
    """Latches on physically implausible odometry; only an explicit reset clears it."""

    def __init__(self,max_linear_speed,max_angular_speed,jump_margin):
        if min(max_linear_speed,max_angular_speed)<=0 or jump_margin<0:
            raise ValueError('invalid divergence bounds')
        self.v_max,self.w_max,self.margin=max_linear_speed,max_angular_speed,jump_margin
        self.reason=None
        self.last=None

    def check(self,stamp,position,velocity,angular):
        if self.reason is not None:
            return False
        speed=float(np.linalg.norm(velocity))
        if speed>self.v_max:
            self.reason=f'linear speed {speed:.2f} m/s exceeds {self.v_max:.2f}'
        elif abs(float(angular[2]))>self.w_max:
            self.reason=f'yaw rate {abs(float(angular[2])):.2f} rad/s exceeds {self.w_max:.2f}'
        elif self.last is not None:
            dt=stamp-self.last[0]
            jump=float(np.linalg.norm(np.asarray(position)-self.last[1]))
            if jump>self.v_max*dt+self.margin:
                self.reason=f'position jumped {jump:.2f} m in {dt:.3f} s'
        if self.reason is not None:
            return False
        self.last=(stamp,np.asarray(position,dtype=float))
        return True

    def reset(self):
        self.reason=None
        self.last=None


def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped, Vector3Stamped
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_srvs.srv import Trigger
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    class Adapter(Node):
        def __init__(self):
            super().__init__('odometry_adapter')
            self.t=self.declare_parameter('base_imu_xyz',[0.,0.,0.]).value
            self.q=self.declare_parameter('base_imu_quat',[0.,0.,0.,1.]).value
            if len(self.t)!=3 or not np.isfinite(self.t).all():
                raise ValueError('invalid base_imu_xyz')
            rotation(self.q)
            self.odom=self.declare_parameter('odom_frame','odom').value
            self.base=self.declare_parameter('base_frame','base_link').value
            self.lio=self.declare_parameter('lio_frame','lio_world').value
            self.align=None
            self.guard=DivergenceGuard(float(self.declare_parameter('max_linear_speed',3.0).value),
                                       float(self.declare_parameter('max_angular_speed',3.0).value),
                                       float(self.declare_parameter('jump_margin',0.3).value))
            self.last=-1
            self.pub=self.create_publisher(Odometry,self.declare_parameter('output_topic','/odom').value,10)
            self.tf=TransformBroadcaster(self)
            self.static_tf=StaticTransformBroadcaster(self)
            latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Vector3Stamped,self.declare_parameter('gravity_topic','/lio/gravity').value,
                                     self.on_gravity,latched)
            self.sub=self.create_subscription(Odometry,self.declare_parameter('input_topic','/lio/odometry').value,self.callback,10)
            self.create_service(Trigger,'~/reset',self.reset)

        def reset(self,_,response):
            previous=self.guard.reason
            self.guard.reset()
            response.success=True
            response.message='cleared: '+previous if previous else 'not latched'
            self.get_logger().warn('Divergence latch reset by operator')
            return response

        def on_gravity(self,msg):
            if self.align is not None:
                return
            if msg.header.frame_id!='camera_init':
                self.get_logger().error('Gravity not in camera_init; refusing alignment')
                return
            try:
                self.align=odom_from_gravity([msg.vector.x,msg.vector.y,msg.vector.z],self.t,self.q)
            except ValueError as e:
                self.get_logger().error(str(e))
                return
            r,p=self.align
            tf=TransformStamped(); tf.header.stamp=msg.header.stamp
            tf.header.frame_id=self.odom; tf.child_frame_id=self.lio
            tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,p)
            tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=map(float,quaternion(r))
            self.static_tf.sendTransform(tf)
            tilt=np.degrees(np.arccos(np.clip(r[2,2],-1,1)))
            self.get_logger().info(f'odom aligned with gravity; camera_init was tilted {tilt:.2f} deg')

        def callback(self,msg):
            if msg.header.frame_id!='camera_init' or msg.child_frame_id!='body':
                self.get_logger().error('Unexpected LIO frames; refusing conversion')
                return
            if self.align is None:
                self.get_logger().warn('Waiting for the LIO gravity estimate; odometry withheld',throttle_duration_sec=2.0)
                return
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            if stamp<=self.last:
                return
            p,q=msg.pose.pose.position,msg.pose.pose.orientation
            v,w=msg.twist.twist.linear,msg.twist.twist.angular
            qi=[q.x,q.y,q.z,q.w]
            try:
                pos,quat,vel,angular=convert_state([p.x,p.y,p.z],qi,[v.x,v.y,v.z],[w.x,w.y,w.z],self.t,self.q)
                r_bi=rotation(self.q)
                t_ib=-r_bi.T @ np.asarray(self.t)
                j=np.eye(6)
                j[:3,3:]=-skew(rotation(qi) @ t_ib)
                pc=j @ np.array(msg.pose.covariance).reshape(6,6) @ j.T
                # camera_init -> odom: world-frame (left-perturbation) pose covariance rotates with R_oc.
                r_oc,p_oc=self.align
                pos,quat=apply_alignment(r_oc,p_oc,pos,quat)
                j=np.zeros((6,6)); j[:3,:3]=r_oc; j[3:,3:]=r_oc
                pc=j @ pc @ j.T
                j=np.zeros((6,6))
                j[:3,:3]=r_bi; j[3:,3:]=r_bi; j[:3,3:]=-r_bi @ skew(t_ib)
                tc=j @ np.array(msg.twist.covariance).reshape(6,6) @ j.T
                if not np.isfinite(pc).all() or not np.isfinite(tc).all():
                    raise ValueError('invalid covariance')
            except ValueError as e:
                self.get_logger().error(str(e))
                return
            self.last=stamp
            if not self.guard.check(stamp*1e-9,pos,vel,angular):
                self.get_logger().error('LIO divergence latched, odometry withheld: '+self.guard.reason,
                                        throttle_duration_sec=2.0)
                return
            out=Odometry(); out.header=msg.header
            out.header.frame_id=self.odom; out.child_frame_id=self.base
            out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=map(float,pos)
            out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=map(float,quat)
            out.twist.twist.linear.x,out.twist.twist.linear.y,out.twist.twist.linear.z=map(float,vel)
            out.twist.twist.angular.x,out.twist.twist.angular.y,out.twist.twist.angular.z=map(float,angular)
            out.pose.covariance=pc.ravel().tolist(); out.twist.covariance=tc.ravel().tolist()
            tf=TransformStamped(); tf.header=out.header; tf.child_frame_id=self.base
            tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,pos)
            tf.transform.rotation=out.pose.pose.orientation
            self.pub.publish(out); self.tf.sendTransform(tf)
    rclpy.init()
    node=Adapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node(); rclpy.try_shutdown()


if __name__=='__main__':
    main()
