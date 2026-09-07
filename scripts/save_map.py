#!/usr/bin/env python3
"""Bounded map-save client. Never accept a stale or empty PCD as success."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import rclpy
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Trigger


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('map_name',nargs='?',default='map')
    parser.add_argument('--convert',action='store_true')
    parser.add_argument('--ground-z',type=float)
    parser.add_argument('--service',default='/uslam/map_save')
    parser.add_argument('--node',default='/fastlio_mapping')
    parser.add_argument('--out-dir',type=Path,default=Path(os.getenv('OUT_DIR',str(Path.home()/'.ros'))))
    args=parser.parse_args()
    if Path(args.map_name).name!=args.map_name or args.map_name in ('.','..'):
        parser.error('map_name must be a filename, not a path')
    if args.convert and args.ground_z is None:
        parser.error('--convert requires --ground-z in the saved map coordinate frame')
    rclpy.init(args=[]); node=rclpy.create_node('universal_map_save_client')
    def call(kind,name,request):
        client=node.create_client(kind,name)
        if not client.wait_for_service(timeout_sec=5): raise RuntimeError('service unavailable: '+name)
        future=client.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=30)
        if not future.done(): raise RuntimeError('service timeout: '+name)
        return future.result()
    try:
        response=call(GetParameters,args.node+'/get_parameters',GetParameters.Request(names=['map_file_path']))
        if not response.values or not response.values[0].string_value: raise RuntimeError('map_file_path not configured')
        path=Path(response.values[0].string_value)
        if not path.is_absolute(): raise RuntimeError('map_file_path must be absolute')
        if os.getenv('PCD_FILE') and Path(os.environ['PCD_FILE']).resolve()!=path.resolve():
            raise RuntimeError('PCD_FILE does not match the running mapper')
        before=path.stat().st_mtime_ns if path.exists() else None
        response=call(Trigger,args.service,Trigger.Request())
        if not response.success: raise RuntimeError('mapper refused save: '+response.message)
        if not path.is_file() or path.stat().st_mtime_ns==before: raise RuntimeError('map was not updated')
        count=0
        with path.open('rb') as f:
            for _ in range(64):
                line=f.readline(4096)
                if line.startswith(b'POINTS '): count=int(line.split()[1])
                if line.startswith(b'DATA '): break
            else: raise RuntimeError('invalid PCD header')
        if count<=0: raise RuntimeError('saved PCD is empty')
        print(f'Saved {count} points: {path}')
    finally:
        node.destroy_node(); rclpy.try_shutdown()
    if not args.convert: return
    args.out_dir.mkdir(parents=True,exist_ok=True)
    topic='/uslam/map_conversion'
    process=subprocess.Popen(['ros2','run','pcd2pgm','pcd2pgm_node','--ros-args',
        '-p','pcd_file:='+str(path),'-p','map_topic_name:='+topic,
        '-p','thre_z_min:='+str(args.ground_z+.15),'-p','thre_z_max:='+str(args.ground_z+2.)],
        start_new_session=True)
    try:
        subprocess.run(['ros2','run','nav2_map_server','map_saver_cli','-t',topic,
            '-f',str(args.out_dir/args.map_name),'--ros-args','-p','save_map_timeout:=20.0'],
            check=True,timeout=30)
    finally:
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL); process.wait()


if __name__=='__main__': main()

