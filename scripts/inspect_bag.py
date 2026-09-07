#!/usr/bin/env python3
"""Read a ROS2 SQLite bag without publishing or modifying it."""
import argparse
import json
import sqlite3
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Imu

parser = argparse.ArgumentParser()
parser.add_argument('bag', type=Path)
args = parser.parse_args()
con = sqlite3.connect(args.bag.resolve().as_uri() + '?mode=ro', uri=True)
result = {}
for topic_id, name, kind in con.execute('SELECT id,name,type FROM topics'):
    count, first, last = con.execute('SELECT COUNT(*), MIN(timestamp), MAX(timestamp) FROM messages WHERE topic_id=?', (topic_id,)).fetchone()
    entry = {'type': kind, 'messages': count, 'duration_s': (last-first)*1e-9 if count else 0}
    if kind not in ('sensor_msgs/msg/PointCloud2', 'sensor_msgs/msg/Imu'):
        result[name] = entry
        continue
    frames, stamps = set(), []
    for blob, in con.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp', (topic_id,)):
        msg = deserialize_message(blob, PointCloud2 if kind.endswith('PointCloud2') else Imu)
        frames.add(msg.header.frame_id)
        stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9)
        if isinstance(msg, PointCloud2):
            entry['fields'] = [{'name': f.name, 'offset': f.offset, 'type': f.datatype, 'count': f.count} for f in msg.fields]
            entry['point_step'], entry['width'], entry['height'] = msg.point_step, msg.width, msg.height
            formats = {1:'i1',2:'u1',3:'i2',4:'u2',5:'i4',6:'u4',7:'f4',8:'f8'}
            dtype = np.dtype({'names':[f.name for f in msg.fields],
                'formats':[('>' if msg.is_bigendian else '<')+formats[f.datatype] for f in msg.fields],
                'offsets':[f.offset for f in msg.fields], 'itemsize':msg.point_step})
            points = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                                strides=(msg.row_step, msg.point_step))
            ranges={f.name:[float(np.nanmin(points[f.name])),float(np.nanmax(points[f.name]))] for f in msg.fields}
            for key,pair in ranges.items():
                old=entry.setdefault('field_ranges',{}).get(key,pair)
                entry['field_ranges'][key]=[min(old[0],pair[0]),max(old[1],pair[1])]
            if 'timestamp' in points.dtype.names:
                duration=float(np.nanmax(points['timestamp'])-np.nanmin(points['timestamp']))
                entry['max_scan_duration_s']=max(entry.get('max_scan_duration_s',0),duration)
        else:
            entry['example_acceleration'] = [msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]
            entry['example_gyro'] = [msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]
    entry['frames'] = sorted(frames)
    entry['sample_header_stamps'] = stamps[:3]
    entry['non_increasing_headers']=int((np.diff(stamps)<=0).sum())
    entry['sample_dt_s'] = [min(np.diff(stamps)),max(np.diff(stamps))] if len(stamps)>1 else []
    result[name] = entry
print(json.dumps(result, indent=2))
con.close()
