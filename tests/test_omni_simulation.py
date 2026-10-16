"""Physical acceptance: lateral drive, measured IMU and solid boundaries."""
from pathlib import Path
import numpy as np
from nav_stack.omni_world import OmniWorld

FIELD = Path(__file__).resolve().parents[1]/'src/nav_stack/worlds/obstacle_arena_10x10.xml'


def test_arena_interior_is_ten_metres_and_sensor_measures_gravity():
    world = OmniWorld(FIELD, rays=600)
    for axis, low, high in [(0, 'west', 'east'), (1, 'south', 'north')]:
        a, b = world.model.geom('wall_'+low), world.model.geom('wall_'+high)
        assert abs((b.pos[axis]-b.size[axis])-(a.pos[axis]+a.size[axis])-10.) < 1e-9
    residuals, rates = [], []
    for _ in range(200):
        acc, gyro = world.step(0., 0., 0.)
        residuals.append(acc-world.transform[:3,:3].T @ [0., 0., 9.81])
        rates.append(gyro)
    np.testing.assert_allclose(np.mean(residuals, axis=0), 0., atol=.025)
    np.testing.assert_allclose(np.mean(rates, axis=0), 0., atol=.002)
    points, ids = world.scan_chunk(np.arange(world.rays))
    assert len(points) > 200 and np.isfinite(points).all()
    assert np.all(np.diff(ids) > 0)


def test_mecanum_strafes_without_turning_or_forward_motion():
    world = OmniWorld(FIELD, rays=100)
    start = np.array(world.pose)
    for _ in range(400):
        world.step(0., .3, 0.)
    delta = np.array(world.pose)-start
    assert .45 < delta[1] < .60
    assert abs(delta[0]) < .02 and abs(delta[2]) < .02
    assert world.collision_steps == 0
    speeds = world.data.qvel[world.wheel_dofs]
    assert speeds[0] < -3 and speeds[1] > 3 and speeds[2] > 3 and speeds[3] < -3


def test_contact_blocks_motion_through_wall():
    world = OmniWorld(FIELD, x=-4.5, y=-3.8, rays=100)
    for _ in range(600):
        world.step(-.3, 0., 0.)
    assert world.collision_steps > 0
    assert -4.80 < world.pose[0] < -4.65

