#!/usr/bin/env python3
"""Terrain-aware obstacle branch for the 2D costmaps.

Works on the gravity-aligned odom cloud (FAST-LIO camera_init mapped through the static
odom -> lio_frame transform of the odometry adapter) with a rolling 2.5D elevation map (see
ElevationMap). Publishes the current scan's vertical obstacles (walls, people) for the ordinary
raytracing obstacle layer, and terrain marks - step edges (curbs, platform and stair edges, ramp
sides) and drops (pits, stairs down, platform edges seen from above) - for a marking-only terrain
layer, and the clearing rays (over observed obstacle-free ground) that alone may erase terrain marks. Ramps below max_slope stay
traversable. Heights are compared locally, so vertical LIO drift and small tilts cancel out.
Not a stair-climbing or rough-terrain planner.
"""
from collections import deque
import math

import numpy as np


def filter_points(xyz, base_xyz, base_height, self_radius, min_height, max_height, max_range=np.inf):
    """Flat-ground cut (kept for reference/tests): heights above the floor under the robot."""
    ground = base_xyz[2] - base_height
    distance = np.hypot(xyz[:, 0] - base_xyz[0], xyz[:, 1] - base_xyz[1])
    keep = (np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > ground + min_height) &
            (xyz[:, 2] < ground + max_height) & (distance > self_radius) & (distance < max_range))
    return xyz[keep]


class ElevationMap:
    """Rolling 2.5D elevation map in the gravity-aligned odom frame; yields the terrain marks.

    Cells (0.1 m) keep the lowest return of the last two scans that saw them and count once both
    exist, with height = the higher of the two (one spurious low return cannot make a hole, while a
    pit wall whose lowest visible point moves with the viewpoint stays low). A cell is forgotten once
    the robot has travelled max_travel since its last observation (vertical LIO drift grows with
    distance; optional max_age), so a robot standing still keeps what it saw on the way, e.g. the
    ramp it climbed or the floor around it, now in its blind zone.

    Step edges (view-independent): cells i, j within `radius` violate traversability if
    h_i - h_j > step_min and h_i - h_j > tan(max_slope) * (d + cell) (a cell's height is its lowest
    return, anywhere inside it) and no cell between them is at the height of i; the HIGHER cell is
    marked (curb, ramp side face, platform or stair edge, box). Ramps below max_slope are free from
    any side and an edge is one cell wide, so a platform barely larger than the robot stays usable.
    Beyond edge_near from the sensor both cells must have been observed within edge_dt of each other:
    the map's attitude residual differs between scans, and cells stitched from different times would
    otherwise show phantom steps (5-9 cm at 5 m for 0.5-1 deg).

    Drops (line of sight): walking out from the sensor along each azimuth, a known cell more than
    `drop` below the ray's current ground and below the robot ground g0 is a drop (pit, stairs down,
    platform edge), and so are the unseen cells back to that ground in which ground would have been
    visible: beyond the blind radius of the lowest beam observed in that azimuth,
    (sensor_z - ground) / tan(-lowest elevation), so the sensor's own blind zone (e.g. the platform
    the robot stands on) is never taken for a drop. Past a drop the lower level is the ray's ground
    (the floor seen from a platform is a drop only at the edge) unless the profile climbs back to
    the level before the drop within `rejoin` m (far rim of a pit). After an occluder (step top,
    vertical obstacle) the ray is not judged until it is back on ground: the gap behind a box is its
    shadow, not a drop, and the far floor seen past it, slightly low through the map's attitude
    residual (legged gait), is not a pit. Drops are remembered until a
    clearing ray runs over the cell or the robot stands on it: a pit rim seen from afar stays marked
    when it is in the blind zone, also in a rolling costmap that has scrolled past it. Only unknown
    cells within support_radius (default half the self radius: under the feet, not the whole body
    circle, which near a pit covers its rim) are taken as the ground the robot stands on.

    Points of the current scan between min_height and max_height above their cell's ground (its
    lowest return of this scan while unconfirmed) are vertical obstacles (walls, people, box faces);
    cells holding such points are not terrain observations, so a person walking by leaves no step.

    Clearing rays (`clear_ends`): per azimuth, how far the ray runs over observed, obstacle-free
    ground only (no drop, step, vertical obstacle or unseen gap > clear_gap on the way; the robot's
    self-masked body zone is not a gap, so a robot standing still clears what moved away beside it).
    They clear
    both costmap layers and the drop memory: a mark disappears only once ground is seen where it was
    (a person who walked away in an open hall, with nothing behind to raytrace to).
    """

    def __init__(self, base_height, self_radius, min_height=0.15, max_height=2.0, size=18.0, cell=0.1,
                 max_slope_deg=15.0, step_min=0.05, drop=0.15, radius=0.3, max_age=None, max_travel=5.0,
                 shadow_max=8.0, sector_deg=1.0, clear_gap=0.2, rejoin=2.0, out_resolution=None, support_radius=None,
                 edge_near=2.5, edge_dt=0.3):
        self.base_height, self.self_radius = base_height, self_radius
        # 机身圆（点云自遮挡半径）内只有脚下核心区一定是地面；贴近坑边时机身圆会盖住坑沿
        self.support = 0.5 * self_radius if support_radius is None else support_radius
        self.min_height, self.max_height, self.cell = min_height, max_height, cell
        self.tan, self.step_min, self.drop = math.tan(math.radians(max_slope_deg)), step_min, drop
        self.max_age, self.max_travel = max_age, max_travel
        self.shadow_max, self.clear_gap, self.rejoin = shadow_max, clear_gap, rejoin
        self.edge_near, self.edge_dt = edge_near, edge_dt
        self.n = int(round(size / cell))
        k = int(round(radius / cell))
        # 每个邻域偏移及其连线上的中间格（用于把台阶边缘限制为一格宽）
        self.offsets = []
        for a in range(-k, k + 1):
            for b in range(-k, k + 1):
                if (a or b) and a * a + b * b <= k * k:
                    m = max(abs(a), abs(b))
                    self.offsets.append((a, b, sorted({(int(round(a * i / m)), int(round(b * i / m))) for i in range(1, m)})))
        # 输出虚拟点按代价地图分辨率铺满整格，否则 0.1 m 格只标到 0.05 m 代价地图的四分之一
        m = max(int(round(cell / out_resolution)), 1) if out_resolution else 1
        self.fill = (np.arange(m) + 0.5) * cell / m - cell / 2
        self.clear_ends = np.empty((0, 3))
        self.h = np.full((self.n, self.n), np.nan)    # lowest return of the latest observation
        self.h2 = np.full((self.n, self.n), np.nan)   # ... of the one before
        self.t = np.full((self.n, self.n), -np.inf)       # time and odometer of the last observation
        self.d = np.full((self.n, self.n), -np.inf)
        self.drops = np.full((self.n, self.n), np.nan)    # remembered drop: height of the low side
        self.odo, self.last_xy = 0.0, None                # path length of the base
        self.origin = None   # odom xy of cell (0, 0) corner
        self.theta = np.radians(np.arange(0.0, 360.0, sector_deg)) - math.pi

    def _recentre(self, bx, by):
        half = self.n * self.cell / 2
        if self.origin is None:
            self.origin = np.array([bx - half, by - half])
            return
        shift = np.round((np.array([bx, by]) - (self.origin + half)) / self.cell).astype(int)
        if np.abs(shift).max() * self.cell < 2.0:
            return
        for arr, fill in ((self.h, np.nan), (self.h2, np.nan), (self.t, -np.inf), (self.d, -np.inf), (self.drops, np.nan)):
            rolled = np.roll(arr, (-shift[0], -shift[1]), axis=(0, 1))
            if shift[0] > 0: rolled[-shift[0]:, :] = fill
            if shift[0] < 0: rolled[:-shift[0], :] = fill
            if shift[1] > 0: rolled[:, -shift[1]:] = fill
            if shift[1] < 0: rolled[:, :-shift[1]] = fill
            arr[...] = rolled
        self.origin = self.origin + shift * self.cell

    def update(self, xyz, base_xyz, sensor_xyz, stamp):
        """Fuse one scan (odom frame; base and sensor origin in odom).

        Returns (vertical_points, edge_points, drop_points); edges and drops are virtual points (cell
        centres filled to out_resolution, z = ground + 0.3)."""
        xyz = np.asarray(xyz, dtype=float)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        bx, by, bz = map(float, base_xyz)
        sx, sy, sz = map(float, sensor_xyz)
        g0 = bz - self.base_height
        xyz = xyz[(np.hypot(xyz[:, 0] - bx, xyz[:, 1] - by) > self.self_radius) & (xyz[:, 2] < g0 + self.max_height + 1.0)]
        # 每个方位扇区实际观测到的最低线束仰角（重力系，含安装倾角），用于求雷达自身盲区半径
        S = len(self.theta)
        dx, dy = xyz[:, 0] - sx, xyz[:, 1] - sy
        sector = np.round((np.arctan2(dy, dx) + math.pi) / (2 * math.pi / S)).astype(np.int64) % S
        emin = np.full(S, np.inf)
        np.minimum.at(emin, sector, np.arctan2(xyz[:, 2] - sz, np.hypot(dx, dy)))
        emin[~np.isfinite(emin)] = emin.min() if np.isfinite(emin).any() else 0.0
        tan_down = np.tan(np.clip(-emin, 0.0, math.radians(89.0)))
        self._recentre(bx, by)
        if self.last_xy is not None:
            self.odo += math.hypot(bx - self.last_xy[0], by - self.last_xy[1])
        self.last_xy = (bx, by)
        n, c = self.n, self.cell
        ij = np.floor((xyz[:, :2] - self.origin) / c).astype(np.int64)
        inside = (ij >= 0).all(axis=1) & (ij < n).all(axis=1)
        xyz, ij = xyz[inside], ij[inside]
        flat = ij[:, 0] * n + ij[:, 1]
        zmin = np.full(n * n, np.inf)
        np.minimum.at(zmin, flat, xyz[:, 2])
        zmin = zmin.reshape(n, n)
        seen = np.isfinite(zmin)
        # 竖直障碍：相对所在格的地面（已确认的高度；未确认的格用本帧该格最低回波，不用机器人脚下地面，
        # 否则初次看到的坡面会被当成障碍，开阔处又没有更远的点把它射线清除掉）
        H_prev = np.fmax(self.h, self.h2)
        H_prev[np.isnan(self.h) | np.isnan(self.h2)] = np.nan
        ci, cj = ij[:, 0], ij[:, 1]
        ref = H_prev[ci, cj]
        hgt = xyz[:, 2] - np.where(np.isnan(ref), zmin[ci, cj], ref)
        vertical = (hgt > self.min_height) & (hgt < self.max_height)
        occupied = np.zeros((n, n), bool)
        occupied[ci[vertical], cj[vertical]] = True
        # 含向上凸出障碍的格（人、墙、箱体侧面：格内最高点高出机器人地面 min_height）不是地形观测：行人腿部的
        # 最低回波会在地形层留下持久的“台阶”，人走到机身自遮挡半径内后再也看不到，机器人被困在身边的残影里。
        # 坑壁、下行台阶的竖直面在地面以下，仍是地形
        zmax = np.full(n * n, -np.inf)
        np.maximum.at(zmax, flat, xyz[:, 2])
        terrain = seen & ~(occupied & (zmax.reshape(n, n) > g0 + self.min_height))
        self.h2[terrain] = self.h[terrain]
        self.h[terrain] = zmin[terrain]
        self.t[terrain] = stamp
        self.d[terrain] = self.odo
        old = self.odo - self.d > self.max_travel
        if self.max_age is not None:
            old |= stamp - self.t > self.max_age
        self.h[old] = np.nan
        self.h2[old] = np.nan
        centre = (np.arange(n) + 0.5) * c
        cx, cy = np.meshgrid(self.origin[0] + centre, self.origin[1] + centre, indexing='ij')
        rc = np.hypot(cx - bx, cy - by)
        H = np.fmax(self.h, self.h2)
        H[np.isnan(self.h) | np.isnan(self.h2)] = np.nan
        # 只有脚下核心区内未知的格视为机器人所站的地面；盲区其余未知格保持未知，否则从未看见过的坑沿
        # 近侧（或机身圆盖住的坑沿）会被当成地面，清除射线随之擦掉代价地图里的落差
        seeded = (rc < self.support) & np.isnan(H)
        H[seeded] = g0
        known = ~np.isnan(H)
        # 边缘：与视角无关的坡度/台阶检验，只标较高的一格
        def shifted(a, b):
            out = np.full((n, n), np.nan)
            out[max(-a, 0):n - max(a, 0), max(-b, 0):n - max(b, 0)] = H[max(a, 0):n - max(-a, 0), max(b, 0):n - max(-b, 0)]
            return out
        Hn = {(a, b): shifted(a, b) for a, b, _ in self.offsets}
        T = self.t
        # 远处只比较观测时间相近的两格：不同时刻的建图姿态残差不同（仿真中约 0.5-1.6 deg），5 m 外即
        # 相差 5-9 cm，跨时刻拼接的相邻格会形成伪台阶；同一帧里的真实路沿两侧同时被观测
        far_cell = np.hypot(cx - sx, cy - sy) > self.edge_near
        edge = np.zeros((n, n), bool)
        with np.errstate(invalid='ignore'):
            for a, b, between in self.offsets:
                Tn = np.full((n, n), -np.inf)
                Tn[max(-a, 0):n - max(a, 0), max(-b, 0):n - max(b, 0)] = T[max(a, 0):n - max(-a, 0), max(b, 0):n - max(-b, 0)]
                # 每格高度取格内最低点，位置在格内不确定：坡度按中心距加一个格宽计，避免稀疏扫描下缓坡误报
                viol = (H - Hn[a, b] > self.step_min) & (H - Hn[a, b] > self.tan * c * (math.hypot(a, b) + 1.0))
                # 中间格与 i 同高说明 i 不在边缘上（平台内部），只有边界格被标记
                for pa, pb in between:
                    viol &= ~(H - Hn[pa, pb] <= self.step_min)
                viol &= ~far_cell | (np.abs(T - Tn) <= self.edge_dt)
                edge |= viol
        edge &= known & ~seeded
        # 负障碍：沿雷达射线在融合后的高程图上推进，记录每条射线上一次的地面
        rs = np.arange(c, n * c / 2, c)
        px = sx + np.cos(self.theta)[:, None] * rs[None, :]
        py = sy + np.sin(self.theta)[:, None] * rs[None, :]
        ai = np.floor((px - self.origin[0]) / c).astype(np.int64)
        bi = np.floor((py - self.origin[1]) / c).astype(np.int64)
        ok = (ai >= 0) & (ai < n) & (bi >= 0) & (bi < n)
        Hs = np.where(ok, H[np.clip(ai, 0, n - 1), np.clip(bi, 0, n - 1)], np.nan)
        body = np.hypot(px - bx, py - by) < self.self_radius   # 机身自遮挡区：永远看不到，不算未观测间隙
        g = np.full(len(self.theta), g0)
        rg = np.full(len(self.theta), c)
        gpre = g.copy()                      # 落差前的地面高度
        rdrop = np.full(len(self.theta), -np.inf)
        neg = np.zeros((n, n), bool)
        run = np.zeros((n, n), bool)             # 清除射线经过的格
        unseen = ok & np.isnan(Hs)
        alive = np.ones(len(self.theta), bool)   # 射线至今只经过已观测地面
        occl = np.zeros(len(self.theta), bool)    # 上一块地面之后经过了遮挡物
        rclear = np.zeros(len(self.theta))
        for k, r in enumerate(rs):
            z = Hs[:, k]
            with np.errstate(invalid='ignore'):
                # 上一块地面之后若经过遮挡物（台阶上沿、竖直障碍），其后的未观测间隙是遮挡而非落差，跨间隙的
                # 高度比较也不可靠（远处地面受姿态残差影响略低）：不判落差，直到射线重新回到地面
                is_neg = (z < g - self.drop) & (z < g0 - self.drop) & ~occl
                is_ground = ~np.isnan(z) & ~is_neg & (z - g <= self.step_min + self.tan * np.minimum(r - rg, 0.3))
                # 落差后又回到落差前的高度（坑的对岸）：恢复原地面基准
                rejoin = (~np.isnan(z) & ~is_neg & ~is_ground & (r - rdrop <= self.rejoin) &
                          (np.abs(z - gpre) <= self.step_min))
            blocked = occupied[np.clip(ai[:, k], 0, n - 1), np.clip(bi[:, k], 0, n - 1)] & ok[:, k]
            back = (is_ground | rejoin) & ~blocked      # 回到地面（含坑对岸）
            occl = (occl | (~np.isnan(z) & ~is_ground & ~is_neg) | blocked) & ~back
            # 清除射线：只走已观测且无竖直障碍的地面，遇落差/台阶/障碍/过长未观测间隙即停
            alive &= ~(~np.isnan(z) & (~is_ground | blocked)) & ~(np.isnan(z) & (r - rclear > self.clear_gap) & ~body[:, k])
            rclear = np.where(alive & is_ground, r, rclear)
            on = alive & is_ground
            run[ai[on, k], bi[on, k]] = True
            for si in np.nonzero(is_neg)[0]:
                neg[ai[si, k], bi[si, k]] = True
                # 只推断“若是地面就应被看到”的未观测格；最低线束够不到的盲区不算落差
                blind = (sz - g[si]) / tan_down[si] if tan_down[si] > 0 else np.inf
                lo = max(int(np.searchsorted(rs, max(rg[si], blind))), k - int(self.shadow_max / c))
                for kk in range(lo, k):
                    if unseen[si, kk]:
                        self.drops[ai[si, kk], bi[si, kk]] = z[si]
            # 落差之后低处成为新的地面基准（从平台上看，地面只在边缘处是落差）
            gpre = np.where(is_neg & (r - rdrop > self.rejoin), g, gpre)
            moved = is_ground | rejoin | is_neg
            g = np.where(moved, z, g)
            rg = np.where(moved, r, rg)
            rdrop = np.where(is_neg, r, rdrop)
        # 落差记忆：清除射线经过（看到了地面）或机器人站上去才遗忘
        self.drops[run | (rc < self.support)] = np.nan
        self.drops[neg] = H[neg]
        drop = np.isfinite(self.drops) & ~seeded
        edge &= ~drop
        far = rclear > 0
        self.clear_ends = np.c_[sx + np.cos(self.theta[far]) * rclear[far], sy + np.sin(self.theta[far]) * rclear[far],
                                np.full(int(far.sum()), g0)]

        def points(mask, height):
            a, b = np.nonzero(mask)
            if not len(a):
                return np.empty((0, 3))
            fx, fy = (f.ravel() for f in np.meshgrid(self.fill, self.fill))
            z = np.nan_to_num(height[a, b], nan=g0) + 0.3
            return np.c_[(cx[a, b][:, None] + fx).ravel(), (cy[a, b][:, None] + fy).ravel(), np.repeat(z, len(fx))]
        return xyz[vertical], points(edge, H), points(drop, self.drops)


def analyze_terrain(xyz, base_xyz, sensor_xyz, base_height, self_radius, min_height=0.15, max_height=2.0, **kw):
    """Single-scan convenience wrapper (fresh map, scan fused twice so its cells are confirmed)."""
    emap = ElevationMap(base_height, self_radius, min_height, max_height, **kw)
    emap.update(xyz, base_xyz, sensor_xyz, 0.0)
    return emap.update(xyz, base_xyz, sensor_xyz, 0.1)


def thin_to_grid(xyz, resolution):
    """One point per resolution-sized x/y column; the 2D costmap only uses x and y."""
    if len(xyz) == 0:
        return xyz
    _, first = np.unique(np.floor(xyz[:, :2] / resolution).astype(np.int64), axis=0, return_index=True)
    return xyz[np.sort(first)]


def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
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
            # Beyond the costmap's obstacle_max_range points are discarded anyway; thinning to the
            # costmap resolution keeps each message small (a few hundred to a few thousand points).
            self.max_range=float(self.declare_parameter('max_range',9.).value)
            self.grid=float(self.declare_parameter('grid_resolution',.05).value)
            self.emap=None
            self.terrain=dict(max_slope_deg=float(self.declare_parameter('max_slope_deg',15.).value),
                              step_min=float(self.declare_parameter('min_step_height',.05).value),
                              drop=float(self.declare_parameter('negative_drop',.15).value),
                              size=2*self.max_range,out_resolution=self.grid)
            self.odom=self.declare_parameter('odom_frame','uslam/odom').value
            self.base=self.declare_parameter('base_frame','uslam/base_link').value
            self.lio=self.declare_parameter('lio_frame','uslam/lio_world').value
            self.lio_to_odom=None
            if (not np.isfinite([self.base_height,self.radius,self.min_h,self.max_h]).all()
                    or self.base_height<=0 or self.radius<=0 or not 0<=self.min_h<self.max_h
                    or not self.max_range>self.radius or not self.grid>0):
                raise ValueError('invalid obstacle filter bounds')
            self.buffer=Buffer(); self.listener=TransformListener(self.buffer,self)
            self.queue=deque(maxlen=5)
            self.pub=self.create_publisher(PointCloud2,self.declare_parameter('output_topic','/uslam/obstacles').value,5)
            # Step edges and drops: marking-only terrain layer, cleared by the clearing rays only.
            self.terrain_pub=self.create_publisher(PointCloud2,self.declare_parameter('terrain_topic','/uslam/terrain_obstacles').value,5)
            # Clearing rays over observed obstacle-free ground, for both costmap layers.
            self.clear_pub=self.create_publisher(PointCloud2,self.declare_parameter('clear_rays_topic','/uslam/clear_rays').value,5)
            self.sensor=self.declare_parameter('sensor_frame','uslam/lidar').value
            self.sub=self.create_subscription(PointCloud2,'/uslam/lio/cloud_registered',self.queue.append,5)
            self.timer=self.create_timer(.05,self.process)

        def process(self):
            if not self.queue: return
            msg=self.queue[-1]
            if msg.header.frame_id!='camera_init': return
            try:
                if self.lio_to_odom is None:
                    s=self.buffer.lookup_transform(self.odom,self.lio,Time())
                    q,t=s.transform.rotation,s.transform.translation
                    x,y,z,w=q.x,q.y,q.z,q.w
                    self.lio_to_odom=(np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                                                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                                                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]),np.array([t.x,t.y,t.z]))
                tf=self.buffer.lookup_transform(self.odom,self.base,Time.from_msg(msg.header.stamp))
                sensor=self.buffer.lookup_transform(self.odom,self.sensor,Time.from_msg(msg.header.stamp))
            except TransformException:
                return
            self.queue.clear()
            fields={f.name:f for f in msg.fields}
            if not all(k in fields and fields[k].datatype==7 for k in ('x','y','z')): return
            dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,
                'offsets':[fields[k].offset for k in ('x','y','z')],'itemsize':msg.point_step})
            if msg.is_bigendian or len(msg.data)!=msg.row_step*msg.height: return
            src=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step))
            xyz=np.column_stack([src[k].ravel() for k in ('x','y','z')]).astype(float)
            r,p=self.lio_to_odom
            xyz=xyz @ r.T+p
            t=tf.transform.translation; st=sensor.transform.translation
            # Circular self mask is yaw-independent; radius must cover legs and payload.
            if self.emap is None:
                self.emap=ElevationMap(self.base_height,self.radius,self.min_h,self.max_h,**self.terrain)
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            vertical,edges,drops=self.emap.update(xyz,(t.x,t.y,t.z),(st.x,st.y,st.z),stamp)
            for pub,pts in ((self.pub,thin_to_grid(vertical,self.grid)),
                            (self.terrain_pub,thin_to_grid(np.vstack([edges,drops]),self.grid)),
                            (self.clear_pub,self.emap.clear_ends)):
                pts=np.asarray(pts,dtype='<f4')
                out=PointCloud2(); out.header.stamp=msg.header.stamp; out.header.frame_id=self.odom
                out.height=1; out.width=len(pts); out.point_step=12; out.row_step=12*out.width
                out.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(('x','y','z'))]
                out.is_dense=True; out.data=pts.tobytes(); pub.publish(out)

    rclpy.init(); node=Filter()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally: node.destroy_node(); rclpy.try_shutdown()


if __name__=='__main__': main()
