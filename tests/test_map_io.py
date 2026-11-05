"""Map round trips, physical coverage and public converter entry points."""
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
from PIL import Image
import pytest
import yaml

from nav_stack.geometry import read_pcd, write_pcd
from nav_stack.map_io import load_occupancy_map, resample_occupancy, save_occupancy_map


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def sample_pcd(tmp_path):
    xs, ys = np.meshgrid(np.arange(0., .7, .025), np.arange(0., .6, .025))
    floor = np.column_stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)])
    wall = np.array([[.35, y, z] for y in np.arange(0., .6, .025)
                     for z in np.arange(.05, .5, .025)])
    path = tmp_path/'maps'/'map.pcd'
    path.parent.mkdir()
    write_pcd(path, np.vstack([floor, wall]))
    return path


@pytest.mark.parametrize('free,occupied', [(.196, .65), (.25, .65), (.001, .002)])
def test_trinary_map_round_trip_keeps_cell_states_and_orientation(tmp_path, free, occupied):
    values = np.array([[-1, 0, 100], [100, -1, 0]], dtype=np.int8)
    if occupied == .002:
        # No 8-bit gray level fits this unknown probability interval.
        with pytest.raises(ValueError, match='8-bit'):
            save_occupancy_map(values, .03, [-2., 3., .7], tmp_path/'map.yaml',
                               free_thresh=free, occupied_thresh=occupied)
        return
    path, image_path = save_occupancy_map(values, .03, [-2., 3., .7],
                                         tmp_path/'nested'/'custom.yaml',
                                         free_thresh=free, occupied_thresh=occupied)
    loaded, resolution, origin = load_occupancy_map(path)
    np.testing.assert_array_equal(loaded, values)
    assert resolution == .03 and image_path.name == 'custom.pgm'
    np.testing.assert_allclose(origin, [-2., 3., .7])


def test_narrow_representable_thresholds_round_trip(tmp_path):
    values = np.array([[-1, 0, 100]], dtype=np.int8)
    path, _ = save_occupancy_map(values, .05, [0., 0., 0.], tmp_path/'map.yaml',
                                  free_thresh=.001, occupied_thresh=.004)
    np.testing.assert_array_equal(load_occupancy_map(path)[0], values)


def test_bundled_map_retains_unknown_cells():
    values, _, _ = load_occupancy_map(ROOT/'maps/map.yaml')
    with Image.open(ROOT/'maps/map.pgm') as image:
        pixels = np.flipud(np.asarray(image))
    assert np.count_nonzero(pixels == 205) > 0
    assert np.all(values[pixels == 205] == -1)


@pytest.mark.parametrize('negate,free,expected', [(0, .196, -1), (0, .25, 0), (1, .25, 100)])
def test_reader_obeys_yaml_instead_of_special_casing_gray205(tmp_path, negate, free, expected):
    Image.fromarray(np.array([[205]], dtype=np.uint8)).save(tmp_path/'external.pgm')
    path = tmp_path/'external.yaml'
    path.write_text(yaml.safe_dump(dict(image='external.pgm', resolution=.05,
        origin=[0, 0, 0], negate=negate, free_thresh=free, occupied_thresh=.65)))
    assert load_occupancy_map(path)[0][0, 0] == expected


def test_yaml_yaw_is_orientation_and_height_stays_zero(tmp_path):
    from nav_stack.localizer_node import load_grid_from_yaml
    yaw = np.pi/3
    path, _ = save_occupancy_map(np.array([[0, -1, 100]]), .05,
                                  [1.2, -3.4, yaw], tmp_path/'map.yaml')
    grid = load_grid_from_yaml(path)
    assert grid.info.origin.position.z == 0.
    np.testing.assert_allclose([grid.info.origin.position.x, grid.info.origin.position.y],
                               [1.2, -3.4])
    q = grid.info.origin.orientation
    np.testing.assert_allclose([q.x, q.y, q.z, q.w], [0., 0., np.sin(yaw/2), np.cos(yaw/2)])
    assert list(grid.data) == [0, -1, 100]


def test_nondivisible_resolution_preserves_obstacle_position_and_unknown_extent():
    # Source extent is 0.4 x 0.1 m; obstacle occupies x=[0.2, 0.3] m.
    source = np.array([[-1, 0, 100, 0]], dtype=np.int8)
    grid = resample_occupancy(source, .1, .03)
    assert grid.shape == (4, 14)  # cover 0.42 x 0.12 m, without scaling source geometry
    np.testing.assert_array_equal(grid[:3], np.tile(
        [-1, -1, -1, -1, 0, 0, 100, 100, 100, 100, 0, 0, 0, -1], (3, 1)))
    np.testing.assert_array_equal(grid[3],
        [-1, -1, -1, -1, -1, -1, 100, 100, 100, 100, -1, -1, -1, -1])


def test_downsampling_preserves_small_obstacles_and_unknown_coverage():
    source = np.array([[0, 0, -1, 0], [0, 100, 0, 0]], dtype=np.int8)
    np.testing.assert_array_equal(resample_occupancy(source, .1, .2), [[100, -1]])


def test_integer_ratio_has_no_spurious_boundary_overlaps():
    source = np.array([[0, 100, -1], [-1, 0, 100]], dtype=np.int8)
    expected = np.repeat(np.repeat(source, 10, axis=0), 10, axis=1)
    np.testing.assert_array_equal(resample_occupancy(source, .1, .01), expected)


@pytest.mark.parametrize('resolution', [0., -1., np.nan, np.inf])
def test_invalid_resolution_is_rejected(tmp_path, resolution):
    with pytest.raises(ValueError, match='resolution'):
        resample_occupancy(np.zeros((2, 2)), .1, resolution)
    with pytest.raises(ValueError, match='resolution'):
        save_occupancy_map(np.zeros((2, 2)), resolution, [0, 0, 0], tmp_path/'map.yaml')


@pytest.mark.parametrize('free,occupied', [(0, .65), (.65, .65), (.7, .65), (.2, 1), (.2, np.nan)])
def test_invalid_thresholds_are_rejected(tmp_path, free, occupied):
    with pytest.raises(ValueError, match='thresholds'):
        save_occupancy_map(np.zeros((2, 2)), .05, [0, 0, 0], tmp_path/'map.yaml',
                           free_thresh=free, occupied_thresh=occupied)


@pytest.mark.parametrize('name', [None, 'office'])
def test_converter_cli_default_input_and_output_names(sample_pcd, name):
    workdir = sample_pcd.parents[1]
    output = workdir/'result'
    cmd = [sys.executable, str(ROOT/'scripts/pcd_to_grid.py'),
           '--output-dir', str(output), '--resolution', '.03']
    if name:
        cmd += ['--name', name]
    result = subprocess.run(cmd, cwd=workdir, capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr
    path = output/f'{name or "map"}.yaml'
    loaded, resolution, _ = load_occupancy_map(path)
    assert resolution == .03
    assert set(np.unique(loaded)) == {-1, 0, 100}
    assert path.with_suffix('.pgm').is_file()


@pytest.mark.parametrize('args,error', [
    (['--resolution', '0'], 'resolution'),
    (['--free-thresh', '.8'], 'thresholds'),
    (['--name', '../outside'], 'filename stem'),
])
def test_converter_cli_reports_invalid_settings_without_traceback(tmp_path, args, error):
    result = subprocess.run([sys.executable, str(ROOT/'scripts/pcd_to_grid.py'), *args],
                            cwd=tmp_path, capture_output=True, text=True, timeout=30)
    assert result.returncode == 2
    assert error in result.stderr and 'Traceback' not in result.stderr


def test_localizer_fallback_creates_named_map_and_retains_unknown(sample_pcd):
    import rclpy
    from nav_stack.localizer_node import PriorMapLocalizer
    path = sample_pcd.parents[1]/'new'/'floor.yaml'
    rclpy.init(args=['--ros-args', '-p', f'map_pcd:={sample_pcd}', '-p', f'map_yaml:={path}'])
    node = None
    try:
        node = PriorMapLocalizer()
        assert path.with_suffix('.pgm').is_file()
        loaded, _, _ = load_occupancy_map(path)
        assert set(np.unique(loaded)) == {-1, 0, 100}
        np.testing.assert_array_equal(np.asarray(node.grid.data).reshape(loaded.shape), loaded)
    finally:
        if node is not None:
            node.pool.shutdown(wait=True)
            node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('destination', ['same', 'symlink', 'separate'])
def test_mapping_save_supports_shared_or_separate_output_directories(sample_pcd, destination):
    from nav_stack.fastlio_map_node import FastLioMapNode
    output = sample_pcd.parents[1]/'run'
    output.mkdir()
    target = output
    if destination == 'symlink':
        target = output.with_name('alias')
        target.symlink_to(output, target_is_directory=True)
    elif destination == 'separate':
        target = output.with_name('saved_maps')
    points = read_pcd(sample_pcd)
    # Exercise the public save operation independently of ROS subscriptions.
    state = SimpleNamespace(output_dir=output, map_save_dir=target,
        get_full_cloud=lambda: points, map_resolution=.03,
        times=[1.], poses=[np.eye(4)], received_clouds=1)
    success, message = FastLioMapNode.save(state)
    assert success, message
    for directory in [output, target]:
        np.testing.assert_allclose(read_pcd(directory/'map.pcd'), points)
        values, resolution, _ = load_occupancy_map(directory/'map.yaml')
        assert resolution == .03 and set(np.unique(values)) == {-1, 0, 100}
    assert (output/'metrics.json').is_file()
    assert (output/'trajectory.tum').is_file()
