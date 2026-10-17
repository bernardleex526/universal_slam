#!/usr/bin/env python3
"""Bounded map-save client. Never accept a stale or empty PCD as success.

--convert writes a Nav2 map_server grid (PGM + YAML). Obstacles are points between
--min-height and --max-height above the LOCAL floor, estimated per cell, so vertical LIO
drift over a long session does not paint the floor (or the robot's own trail) as obstacles.
"""
import argparse
import os
from pathlib import Path

import numpy as np

PCD_TYPES = {('F', 4): '<f4', ('F', 8): '<f8', ('U', 1): 'u1', ('U', 2): '<u2', ('U', 4): '<u4',
             ('I', 1): 'i1', ('I', 2): '<i2', ('I', 4): '<i4'}


def read_pcd_xyz(path):
    """Read x/y/z from an ASCII or binary (uncompressed) PCD file."""
    header = {}
    with open(path, 'rb') as f:
        for _ in range(64):
            line = f.readline(4096).decode('ascii', 'replace').strip()
            if not line or line.startswith('#'):
                continue
            key, *values = line.split()
            header[key] = values
            if key == 'DATA':
                break
        else:
            raise RuntimeError('invalid PCD header')
        names, sizes, types = header['FIELDS'], list(map(int, header['SIZE'])), header['TYPE']
        counts = list(map(int, header.get('COUNT', ['1'] * len(names))))
        points = int(header['POINTS'][0])
        if header['DATA'][0] == 'ascii':
            data = np.loadtxt(f, ndmin=2)
            cols = np.cumsum([0] + counts)
            return np.column_stack([data[:, cols[names.index(k)]] for k in 'xyz']).astype(np.float64)
        if header['DATA'][0] != 'binary':
            raise RuntimeError('compressed PCD is not supported')
        dtype = np.dtype([(n if c == 1 else f'{n}_{i}', PCD_TYPES[(t, s)])
                          for n, s, t, c in zip(names, sizes, types, counts) for i in range(c)])
        cloud = np.frombuffer(f.read(points * dtype.itemsize), dtype=dtype, count=points)
    return np.column_stack([cloud[k].astype(np.float64) for k in 'xyz'])


def _neighbourhood_min(grid, radius):
    padded = np.pad(grid, radius, constant_values=np.inf)
    out = grid.copy()
    h, w = grid.shape
    for dx in range(2 * radius + 1):
        for dy in range(2 * radius + 1):
            out = np.fmin(out, padded[dx:dx + h, dy:dy + w])
    return out


def occupancy_grid(xyz, resolution=0.05, floor_cell=1.0, floor_radius=1, floor_percentile=5.0,
                   min_height=0.15, max_height=2.0):
    """Return (grid, origin_xy). grid: 0 free, 100 occupied, -1 unknown (rows = +y, cols = +x)."""
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if len(xyz) == 0:
        raise RuntimeError('map has no finite points')
    lo = xyz[:, :2].min(axis=0)
    # Local floor: low percentile per coarse cell, then the min over neighbours so a wall or
    # box filling a cell does not raise its floor estimate.
    cidx = np.floor((xyz[:, :2] - lo) / floor_cell).astype(int)
    cshape = cidx.max(axis=0) + 1
    floor = np.full(cshape, np.nan)
    order = np.lexsort((xyz[:, 2], cidx[:, 1], cidx[:, 0]))
    keys = cidx[order]
    starts = np.r_[0, np.flatnonzero((np.diff(keys, axis=0) != 0).any(axis=1)) + 1, len(order)]
    for a, b in zip(starts[:-1], starts[1:]):
        floor[tuple(keys[a])] = np.percentile(xyz[order[a:b], 2], floor_percentile)
    floor = _neighbourhood_min(np.where(np.isnan(floor), np.inf, floor), floor_radius)
    height = xyz[:, 2] - floor[cidx[:, 0], cidx[:, 1]]
    fidx = np.floor((xyz[:, :2] - lo) / resolution).astype(int)
    shape = fidx.max(axis=0) + 1
    grid = np.full(shape, -1, dtype=np.int8)
    ground = height <= min_height
    grid[fidx[ground, 0], fidx[ground, 1]] = 0
    obstacle = (height > min_height) & (height < max_height)
    grid[fidx[obstacle, 0], fidx[obstacle, 1]] = 100
    return grid.T, lo


def write_map(grid, origin, resolution, stem):
    """Nav2 map_server trinary PGM + YAML (row 0 of the image is the max-y row)."""
    image = np.full(grid.shape, 205, dtype=np.uint8)
    image[grid == 0] = 254
    image[grid == 100] = 0
    image = image[::-1]
    stem = Path(stem)
    with open(stem.with_suffix('.pgm'), 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (image.shape[1], image.shape[0]))
        f.write(image.tobytes())
    stem.with_suffix('.yaml').write_text(
        f'image: {stem.name}.pgm\nmode: trinary\nresolution: {resolution}\n'
        f'origin: [{origin[0]:.3f}, {origin[1]:.3f}, 0.0]\nnegate: 0\n'
        f'occupied_thresh: 0.65\nfree_thresh: 0.25\n')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('map_name',nargs='?',default='map')
    parser.add_argument('--convert',action='store_true')
    parser.add_argument('--resolution',type=float,default=0.05)
    parser.add_argument('--min-height',type=float,default=0.15,help='obstacle threshold above local floor (m)')
    parser.add_argument('--max-height',type=float,default=2.0,help='ignore points higher than this above local floor (m)')
    parser.add_argument('--service',default='/uslam/map_save')
    parser.add_argument('--node',default='/fastlio_mapping')
    parser.add_argument('--out-dir',type=Path,default=Path(os.getenv('OUT_DIR',str(Path.home()/'.ros'))))
    args=parser.parse_args()
    if Path(args.map_name).name!=args.map_name or args.map_name in ('.','..'):
        parser.error('map_name must be a filename, not a path')
    if not (args.resolution>0 and 0<=args.min_height<args.max_height):
        parser.error('invalid grid resolution or height band')
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from std_srvs.srv import Trigger
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
    grid,origin=occupancy_grid(read_pcd_xyz(path),args.resolution,min_height=args.min_height,max_height=args.max_height)
    write_map(grid,origin,args.resolution,args.out_dir/args.map_name)
    print(f'Saved grid {grid.shape[1]}x{grid.shape[0]} @ {args.resolution} m: {args.out_dir/args.map_name}.yaml')


if __name__=='__main__': main()
