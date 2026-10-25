"""Local terrain analysis on synthetic LiDAR scans of height fields (sensor 0.6 m above the floor)."""
import importlib.util
import math
from pathlib import Path

import numpy as np

spec = importlib.util.spec_from_file_location('obstacle_filter_terrain', Path(__file__).parents[1] / 'scripts/obstacle_filter.py')
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

BASE = (0.0, 0.0, 0.47)
SENSOR = np.array([0.1, 0.0, 0.60])


def scan(height, sensor=SENSOR):
    """Ray-march a 32-line, 1-degree spinning LiDAR at `sensor` against z = height(x, y)."""
    elev = np.radians(np.linspace(-25, 15, 32))
    az = np.radians(np.arange(0, 360, 1.0))
    d = np.stack([np.cos(elev)[None] * np.cos(az)[:, None], np.cos(elev)[None] * np.sin(az)[:, None],
                  np.broadcast_to(np.sin(elev)[None], (len(az), len(elev)))], -1).reshape(-1, 3)
    t = np.arange(0.4, 10.0, 0.02)
    p = sensor + d[:, None, :] * t[None, :, None]
    below = p[..., 2] < height(p[..., 0], p[..., 1])
    hit = below.any(axis=1)
    k = below.argmax(axis=1)
    return p[np.nonzero(hit)[0], k[hit]]


def positive_and_drops(result):
    """(vertical + step edges, drops) - the two things a test asks about."""
    vertical, edges, drops = result
    return np.vstack([vertical, edges]), drops


def run(height, **kw):
    return run_points(scan(height), **kw)


def run_points(points, **kw):
    return positive_and_drops(mod.analyze_terrain(points, BASE, SENSOR, 0.47, 0.55, **kw))


def box(x, y, x0, x1, y0, y1):
    return (x >= x0) & (x < x1) & (y >= y0) & (y < y1)


def count(points, x0, x1, y0, y1):
    return int(box(points[:, 0], points[:, 1], x0, x1, y0, y1).sum()) if len(points) else 0


def test_flat_floor_is_clean():
    obs, neg = run(lambda x, y: np.zeros_like(x))
    assert len(obs) == 0 and len(neg) == 0


def test_ramp_and_platform_are_traversable_but_box_on_ramp_is_not():
    tan10 = math.tan(math.radians(10))

    def ramp(x, y, with_box=False):
        h = np.where(np.abs(y) < 1.5, np.clip((x - 2.0) * tan10, 0.0, 0.6), 0.0)
        if with_box:
            h = h + np.where(box(x, y, 4.0, 4.4, -0.2, 0.2), 0.3, 0.0)
        return h
    obs, neg = run(ramp)
    assert count(obs, 2.2, 6.5, -1.1, 1.1) == 0, 'ramp surface and platform must stay free (edges may reach 0.3 m in)'
    assert count(neg, 2.2, 6.5, -1.2, 1.2) == 0
    obs, _ = run(lambda x, y: ramp(x, y, True))
    assert count(obs, 3.9, 4.5, -0.3, 0.3) > 0, 'a 0.3 m box on the ramp is measured against the ramp'
    assert count(obs, 2.2, 3.8, -1.2, 1.2) == 0


def test_curb_is_an_obstacle_small_bump_is_not():
    obs, _ = run(lambda x, y: np.where(box(x, y, 3.0, 4.0, -1.0, 1.0), 0.12, 0.0))
    assert count(obs, 2.9, 4.1, -1.0, 1.0) > 0
    obs, neg = run(lambda x, y: np.where(box(x, y, 3.0, 4.0, -1.0, 1.0), 0.05, 0.0))
    assert len(obs) == 0 and len(neg) == 0


def test_pit_is_negative_including_its_unseen_near_part():
    obs, neg = run(lambda x, y: np.where(box(x, y, 3.0, 4.5, -1.0, 1.0), -0.5, 0.0))
    assert count(neg, 3.0, 3.6, -0.3, 0.3) > 0, 'shadowed near part of the pit'
    assert count(neg, 4.0, 4.5, -0.3, 0.3) > 0, 'visible far wall of the pit'
    # up to ~one ring spacing (0.3 m) of floor before the rim may be shadowed too: conservative
    outside = len(neg) - count(neg, 2.7, 4.7, -1.2, 1.2)
    assert outside == 0, 'no negative marks away from the pit'


def test_stairs_down_are_negative():
    def stairs(x, y):
        k = np.clip(np.floor((x - 3.0) / 0.3) + 1, 0, 5)
        return np.where(np.abs(y) < 1.0, -0.17 * k, 0.0)
    _, neg = run(stairs)
    assert count(neg, 3.0, 4.5, -0.5, 0.5) > 0
    assert count(neg, 0.0, 2.7, -1.2, 1.2) == 0


def test_single_spurious_low_return_is_not_a_negative_obstacle():
    floor = scan(lambda x, y: np.zeros_like(x))
    spike = np.array([[3.05, 0.05, -0.6], [3.07, 0.06, -0.6]])
    emap = mod.ElevationMap(0.47, 0.55)
    *_, neg = emap.update(np.r_[floor, spike], BASE, SENSOR, 0.0)
    assert len(neg) == 0, 'one observation of a cell is not enough'
    *_, neg = emap.update(floor, BASE, SENSOR, 0.1)
    assert len(neg) == 0, 'the floor seen again overrides the spurious low return'


def test_ramp_seen_from_the_side_is_traversable_and_its_side_is_an_edge():
    tan10 = math.tan(math.radians(10))
    # ramp rising along +y (x 2..4), robot east of it looking west at its side face
    obs, neg = run(lambda x, y: np.where(box(x, y, 2.0, 4.0, -3.0, 3.0), np.clip((y + 3.0) * tan10, 0, 1.0), 0.0))
    assert count(obs, 2.4, 3.6, -1.0, 2.5) == 0, 'ramp interior is traversable from any side (edges may reach 0.3 m in)'
    assert count(obs, 1.8, 2.2, 0.0, 2.5) > 0, 'the visible (west) side face, > 0.05 m high, is an edge'
    assert len(neg) == 0


def test_platform_wider_than_its_ramp_is_ground_where_the_ramp_leads():
    tan10 = math.tan(math.radians(10))

    def scene(x, y):
        ramp = np.where(np.abs(y) < 0.75, np.clip((x - 2.0) * tan10, 0.0, 0.6), 0.0)
        platform = np.where(box(x, y, 2.0 + 0.6 / tan10, 7.0, -2.0, 2.0), 0.6, 0.0)
        return np.maximum(ramp, platform)
    obs, neg = run(scene)
    assert count(obs, 5.6, 6.6, -0.4, 0.4) == 0, 'platform top straight up the ramp is reachable ground'
    assert count(obs, 5.3, 5.5, 1.0, 2.0) > 0, 'the platform side face seen from the floor is still a step'


def test_pit_rim_stays_negative_when_the_robot_reaches_it():
    """Regression (closed loop): the near rim of a pit is in the lidar blind zone once the robot is
    next to it; it must stay marked (drop memory), not vanish when its inference expires."""
    pit = lambda x, y: np.where(box(x, y, 3.0, 4.5, -1.0, 1.0), -0.5, 0.0)
    emap = mod.ElevationMap(0.47, 0.55)
    for k, bx in enumerate((0.0, 0.0, 0.6, 1.2, 1.8, 2.2)):  # drive towards the pit, last stop 0.8 m from it
        sensor = np.array([bx + 0.1, 0.0, 0.60])
        *_, neg = emap.update(scan(pit, sensor), (bx, 0.0, 0.47), sensor, 0.1 * k)
    assert count(neg, 3.0, 3.3, -0.3, 0.3) > 0, 'the near rim inside the blind zone is still negative'


def platform_scene(x, y):
    """2 m wide ramp rising along +y (y -1..1, 0 -> 0.3 m) onto a 2.0 x 1.5 m platform (y 1..2.5)."""
    ramp = np.where(box(x, y, -1.0, 1.0, -1.0, 1.0), (y + 1.0) * 0.15, 0.0)
    return np.where(box(x, y, -1.0, 1.0, 1.0, 2.5), 0.3, ramp)


def test_robot_on_a_small_platform_keeps_it_free_and_sees_the_ramp_down_as_ground():
    """Regression (closed loop): a 0.3 m wide edge band plus the 0.55 m inscribed radius covered the
    whole 2.0 x 1.5 m platform, so the planner had no valid start once the robot stood on it."""
    base = (0.0, 1.75, 0.3 + 0.47)
    sensor = np.array([0.0, 1.65, 0.3 + 0.60])
    obs, neg = positive_and_drops(mod.analyze_terrain(scan(platform_scene, sensor), base, sensor, 0.47, 0.55))
    assert count(obs, -0.9, 0.9, -1.0, 2.3) == 0, 'platform interior and ramp surface stay free'
    # the boundary cells of the ramp sides (x = +-0.95) lie next to a real drop and may be marked
    assert count(neg, -0.9, 0.9, -1.0, 2.5) == 0, 'the blind zone on the platform and the ramp down are not drops'
    assert count(neg, -1.0, 1.0, 2.5, 4.0) > 0 and count(neg, 1.0, 2.5, 1.0, 2.5) > 0, 'floor behind the edges is a drop'
    far = np.hypot(neg[:, 0] - sensor[0], neg[:, 1] - sensor[1]) > 3.5
    assert not far.any(), 'the floor itself, away from the edges, is ground at a lower level'


def test_clearing_rays_cover_observed_ground_and_stop_before_a_pit():
    pit = lambda x, y: np.where(box(x, y, 3.0, 4.5, -1.0, 1.0), -0.5, 0.0)
    emap = mod.ElevationMap(0.47, 0.55)
    for k, bx in enumerate(np.arange(-3.0, 0.01, 0.3)):   # drive towards the pit
        sensor = np.array([bx + 0.1, 0.0, 0.60])
        *_, neg = emap.update(scan(pit, sensor), (bx, 0.0, 0.47), sensor, 0.1 * k)
    assert count(neg, 3.0, 4.5, -0.5, 0.5) > 0
    ends = emap.clear_ends
    reach = np.hypot(ends[:, 0] - 0.1, ends[:, 1])
    assert len(ends) > 300 and np.median(reach) > 1.5, 'observed floor around the robot is clearable'
    towards = np.abs(ends[:, 1]) < 0.9
    assert towards.any() and ends[towards, 0].max() < 3.0, 'no clearing ray reaches into the pit'
    assert not box(ends[:, 0], ends[:, 1], 3.0, 4.5, -1.0, 1.0).any()


def test_virtual_cells_fill_the_costmap_resolution():
    curb = lambda x, y: np.where(box(x, y, 3.0, 4.0, -1.0, 1.0), 0.12, 0.0)
    coarse, _ = run(curb)
    fine, _ = run(curb, out_resolution=0.05)
    assert len(fine) == 4 * len(coarse)
    cells = np.unique(np.floor(fine[:, :2] / 0.05).astype(int), axis=0)
    assert len(cells) == len(fine), 'one virtual point per 0.05 m costmap cell'


def test_a_second_pit_behind_the_first_is_still_seen():
    pit2 = lambda x, y: np.where(box(x, y, 3.6, 4.6, -1.0, 1.0), -0.5, 0.0)
    _, alone = run(pit2)
    _, neg = run(lambda x, y: np.minimum(pit2(x, y), np.where(box(x, y, 2.0, 2.6, -1.0, 1.0), -0.5, 0.0)))
    assert count(neg, 1.9, 2.7, -0.5, 0.5) > 0
    assert count(neg, 2.75, 4.7, -1.0, 1.0) == count(alone, 2.75, 4.7, -1.0, 1.0) > 0,         'the far rim of the first pit restores the floor reference: what lies behind it is unchanged'


def test_a_robot_standing_still_keeps_the_ramp_it_climbed():
    """Cells expire by distance travelled: after a long stop on the platform the ramp cells seen on
    the way up (now inside the blind zone) still make the ramp ground, not a drop."""
    emap = mod.ElevationMap(0.47, 0.55)
    k = 0
    for by in np.arange(-3.0, 1.76, 0.25):                  # climb the ramp onto the platform
        g = float(platform_scene(np.array([0.0]), np.array([by]))[0])
        sensor = np.array([0.0, by + 0.1, g + 0.60])
        emap.update(scan(platform_scene, sensor), (0.0, by, g + 0.47), sensor, 0.1 * k); k += 1
    sensor = np.array([0.0, 1.65, 0.9])
    pts = scan(platform_scene, sensor)
    for t in (40.0, 40.1):                                  # 40 s stop (old 10 s expiry), facing back down
        obs, neg = positive_and_drops(emap.update(pts, (0.0, 1.75, 0.77), sensor, t))
    assert count(neg, -0.9, 0.9, -1.0, 2.5) == 0
    assert count(obs, -0.9, 0.9, -1.0, 2.3) == 0


def test_remembered_drops_survive_travel_next_to_the_pit_and_clear_when_ground_is_seen():
    pit = lambda x, y: np.where(box(x, y, 3.0, 4.5, -1.0, 1.0), -0.5, 0.0)
    emap = mod.ElevationMap(0.47, 0.55)
    k = 0
    for bx in np.arange(0.0, 2.21, 0.3):                      # approach to 0.8 m of the rim
        sensor = np.array([bx + 0.1, 0.0, 0.60])
        emap.update(scan(pit, sensor), (bx, 0.0, 0.47), sensor, 0.1 * k); k += 1
    sensor = np.array([2.3, 0.0, 0.60])
    pts = scan(pit, sensor)
    for i in range(80):                                       # jitter in place: > max_travel of odometry
        dy = 0.05 * (-1) ** i
        *_, neg = emap.update(pts, (2.2, dy, 0.47), sensor, 0.1 * k); k += 1
    assert emap.odo > emap.max_travel
    assert count(neg, 3.0, 3.4, -0.5, 0.5) > 0, 'the near rim in the blind zone is still a drop'


def test_a_remembered_drop_clears_only_where_ground_is_seen():
    flat = lambda x, y: np.zeros_like(x)
    emap = mod.ElevationMap(0.47, 0.55)
    k = 0
    for bx in np.arange(-3.0, 0.01, 0.3):
        sensor = np.array([bx + 0.1, 0.0, 0.60])
        emap.update(scan(flat, sensor), (bx, 0.0, 0.47), sensor, 0.1 * k); k += 1
    cell = lambda x, y: (int((x - emap.origin[0]) // emap.cell), int((y - emap.origin[1]) // emap.cell))
    for x, y in ((1.8, 0.05), (1.5, 0.75), (8.0, 0.05)):     # stale drops: near on observed floor, and far
        emap.drops[cell(x, y)] = -0.5
    *_, neg = emap.update(scan(flat, SENSOR), BASE, SENSOR, 0.1 * k)
    assert count(neg, 1.4, 1.9, -0.1, 0.9) == 0, 'continuous observed ground clears the memory'
    assert count(neg, 7.9, 8.1, 0.0, 0.1) > 0, 'beyond the clearing rays the drop stays'


def test_terrain_marks_and_vertical_points_are_separate():
    curb_and_wall = lambda x, y: np.where(box(x, y, 3.0, 4.0, -1.0, 1.0), 0.12, 0.0) + np.where(box(x, y, 6.0, 6.2, -3, 3), 2.0, 0.0)
    vertical, edges, drops = mod.analyze_terrain(scan(curb_and_wall), BASE, SENSOR, 0.47, 0.55)
    assert count(edges, 2.9, 4.1, -1.0, 1.0) > 0 and count(vertical, 2.9, 4.1, -1.0, 1.0) == 0, 'curb: terrain layer'
    assert count(vertical, 5.9, 6.3, -2.0, 2.0) > 0, 'wall: ordinary obstacle layer'
    assert len(drops) == 0


def test_a_body_overhanging_a_pit_does_not_seed_its_rim_as_ground():
    """Robot centre 0.4 m from the rim (inside its 0.55 m body circle): the rim cells under the body
    were seeded as ground and the clearing rays then ran across them, erasing the pit."""
    pit = lambda x, y: np.where(box(x, y, 1.4, 3.0, -1.0, 1.0), -0.5, 0.0)
    emap = mod.ElevationMap(0.47, 0.55)
    k = 0
    for bx in np.arange(-3.0, 1.01, 0.25):                    # creep up to 0.4 m from the rim
        sensor = np.array([bx + 0.1, 0.0, 0.60])
        *_, neg = emap.update(scan(pit, sensor), (bx, 0.0, 0.47), sensor, 0.1 * k); k += 1
    assert count(neg, 1.4, 1.6, -0.3, 0.3) > 0, 'rim under the body circle stays a drop'
    ends = emap.clear_ends
    assert not box(ends[:, 0], ends[:, 1], 1.4, 3.0, -1.0, 1.0).any(), 'no clearing ray into the pit'


def test_a_ramp_seen_for_the_first_time_is_not_a_vertical_obstacle():
    """Unconfirmed cells used the robot's ground as reference: the ramp surface 0.15-0.6 m above the
    floor, seen once, became wall points that nothing behind it would ever raytrace away."""
    tan10 = math.tan(math.radians(10))
    ramp = lambda x, y: np.where(np.abs(y) < 1.5, np.clip((x - 2.0) * tan10, 0.0, 0.6), 0.0)
    emap = mod.ElevationMap(0.47, 0.55)
    vertical, _, _ = emap.update(scan(ramp), BASE, SENSOR, 0.0)      # single scan: nothing confirmed
    assert count(vertical, 2.0, 6.0, -1.4, 1.4) == 0


def test_clearing_rays_stop_at_a_person_and_pass_once_it_is_gone():
    flat = lambda x, y: np.zeros_like(x)
    person = np.array([[2.0 + 0.1 * np.cos(a), 0.1 * np.sin(a), z] for a in np.linspace(0, 6.2, 24) for z in np.arange(0.05, 1.7, 0.1)])
    emap = mod.ElevationMap(0.47, 0.55)
    k = 0
    for bx in np.arange(-3.0, 0.01, 0.3):
        sensor = np.array([bx + 0.1, 0.0, 0.60])
        emap.update(scan(flat, sensor), (bx, 0.0, 0.47), sensor, 0.1 * k); k += 1
    vertical, _, _ = emap.update(np.r_[scan(flat), person], BASE, SENSOR, 0.1 * k); k += 1
    assert count(vertical, 1.8, 2.2, -0.2, 0.2) > 0
    ends = emap.clear_ends
    ahead = np.abs(np.arctan2(ends[:, 1], ends[:, 0] - SENSOR[0])) < np.radians(1.5)
    assert ahead.any() and ends[ahead, 0].max() < 1.95, 'no clearing through the person'
    emap.update(scan(flat), BASE, SENSOR, 0.1 * k)
    ends = emap.clear_ends
    ahead = np.abs(np.arctan2(ends[:, 1], ends[:, 0] - SENSOR[0])) < np.radians(1.5)
    assert ends[ahead, 0].max() > 2.2, 'the person is gone: the ray clears its old cell'


def test_a_slight_attitude_error_does_not_turn_the_far_floor_into_drops():
    """Closed loop: a ~1 deg roll/pitch residual of the map (legged gait) put the floor 8 m away 0.18 m
    'below' the robot; behind a box's shadow that read as a drop and the shadow edge became a pit."""
    def scene(x, y):
        return np.where(box(x, y, 2.0, 2.6, -0.3, 0.3), 0.8, 0.0)          # a box casting a long shadow
    a = math.radians(1.2)                                                 # map pitched: far floor ahead lower
    pts = scan(scene) - SENSOR
    pts = np.c_[pts[:, 0] * math.cos(a) + pts[:, 2] * math.sin(a), pts[:, 1],
                -pts[:, 0] * math.sin(a) + pts[:, 2] * math.cos(a)] + SENSOR
    _, neg = run_points(pts)
    assert len(neg) == 0
    pit = lambda x, y: np.where(box(x, y, 2.0, 3.5, -1.0, 1.0), -0.5, 0.0)
    _, neg = run(pit)
    assert count(neg, 2.0, 3.5, -0.5, 0.5) > 0, 'a nearby pit is still a drop'


def test_far_cells_stitched_from_different_times_make_no_phantom_step():
    """Localization run: a 1 deg attitude change between scans lifted a 5 m-away corridor patch by
    8.5 cm against cells seen earlier; the seam became a step edge across the corridor."""
    def patch(x0, x1, z):
        g = np.mgrid[x0:x1:0.03, -0.5:0.5:0.03].reshape(2, -1).T
        return np.c_[g, np.full(len(g), z)]
    floor_near = scan(lambda x, y: np.zeros_like(x))
    floor_near = floor_near[np.hypot(floor_near[:, 0], floor_near[:, 1]) < 4.0]
    emap = mod.ElevationMap(0.47, 0.55)
    for t in (0.0, 0.1):                                       # first half of the patch, then...
        emap.update(np.r_[floor_near, patch(4.5, 5.0, 0.0)], BASE, SENSOR, t)
    for t in (1.0, 1.1):                                       # ...the other half, a second later, 8.5 cm higher
        _, edges, _ = emap.update(np.r_[floor_near, patch(5.0, 5.5, 0.085)], BASE, SENSOR, t)
    assert count(edges, 4.4, 5.6, -0.5, 0.5) == 0
    emap = mod.ElevationMap(0.47, 0.55)
    for t in (0.0, 0.1):                                       # a real 12 cm curb at 5 m, seen in one scan
        _, edges, _ = emap.update(np.r_[floor_near, patch(4.5, 5.0, 0.0), patch(5.0, 5.5, 0.12)], BASE, SENSOR, t)
    assert count(edges, 4.9, 5.2, -0.5, 0.5) > 0


def test_a_person_walking_by_leaves_no_terrain_step_and_a_still_robot_clears_beside_it():
    """People closed loop: leg cells (floor hidden, lowest return 8 cm up) became persistent step
    edges; once the person was inside the self mask the robot sat in its ghost ("start in lethal")."""
    flat = lambda x, y: np.zeros_like(x)
    emap = mod.ElevationMap(0.47, 0.55)
    k = 0
    for bx in np.arange(-3.0, 0.01, 0.3):                           # the robot drives in...
        sensor = np.array([bx + 0.1, 0.0, 0.60])
        emap.update(scan(flat, sensor), (bx, 0.0, 0.47), sensor, 0.1 * k); k += 1
    k += 1000                                                       # ...and stands still for 100 s
    floor = scan(flat)
    for px in (1.6, 1.3, 1.0, 0.8):                                 # a person walks past, ends next to it
        legs = np.array([[px + 0.15 * np.cos(a), 0.6 + 0.15 * np.sin(a), z]
                         for a in np.linspace(0, 6.2, 20) for z in np.arange(0.08, 1.7, 0.1)])
        hidden = np.hypot(floor[:, 0] - px, floor[:, 1] - 0.6) < 0.35
        _, edges, drops = emap.update(np.r_[floor[~hidden], legs], BASE, SENSOR, 0.1 * k); k += 1
        assert count(edges, px - 0.4, px + 0.4, 0.2, 1.0) == 0 and len(drops) == 0
    for _ in range(2):                                              # gone
        _, edges, _ = emap.update(floor, BASE, SENSOR, 0.1 * k); k += 1
    assert len(edges) == 0
    reach = np.hypot(emap.clear_ends[:, 0] - SENSOR[0], emap.clear_ends[:, 1] - SENSOR[1])
    assert np.median(reach) > 1.5, 'clearing rays start through the self-masked body zone of a still robot'
