"""Exercise map selection at the shell entrypoints without starting ROS."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(params=['navigation', 'relocalize'])
def launcher(tmp_path, request):
    workspace = tmp_path / 'workspace'
    scripts = workspace / 'scripts'
    scripts.mkdir(parents=True)
    for name in (f'{request.param}.sh', 'prior_map_args.sh'):
        shutil.copyfile(ROOT / 'scripts' / name, scripts / name)
    (scripts / 'env.sh').write_text('# No ROS installation needed for argument tests.\n')
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    ros2 = bin_dir / 'ros2'
    ros2.write_text(
        f'#!{sys.executable}\n'
        'import json, os, pathlib, sys\n'
        'pathlib.Path(os.environ["ROS_ARGS_LOG"]).write_text(json.dumps(sys.argv[1:]))\n'
    )
    ros2.chmod(0o755)
    call_log = tmp_path / 'ros2-args.json'
    env = dict(os.environ, PATH=f'{bin_dir}:{os.environ["PATH"]}',
               ROS_ARGS_LOG=str(call_log))

    def run(*args):
        result = subprocess.run(
            ['bash', str(scripts / f'{request.param}.sh'), *args],
            cwd=tmp_path, env=env, capture_output=True, text=True, check=False,
        )
        forwarded = json.loads(call_log.read_text()) if call_log.exists() else None
        return result, forwarded

    return workspace, run, request.param


def make_map(path):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('Map file existence is checked by the shell wrapper.\n')
    return path


def test_default_map_is_forwarded(launcher):
    workspace, run, mode = launcher
    pcd = make_map(workspace / 'maps/map.pcd')
    yaml = make_map(pcd.with_suffix('.yaml'))
    result, args = run('rviz:=false')
    assert result.returncode == 0, result.stderr
    assert args[:3] == ['launch', 'nav_stack', f'{mode}.launch.py']
    assert f'map_pcd:={pcd}' in args
    assert f'map_yaml:={yaml}' in args
    assert 'rviz:=false' in args


def test_custom_pcd_without_default_uses_same_stem_yaml(launcher):
    workspace, run, _ = launcher
    pcd = make_map(workspace / 'custom maps/site survey.pcd')
    result, args = run(f'map_pcd:={pcd}', 'gui:=false', 'output_dir:=logs with spaces')
    assert result.returncode == 0, result.stderr
    assert f'map_pcd:={pcd}' in args
    assert f'map_yaml:={pcd.with_suffix(".yaml")}' in args
    assert 'gui:=false' in args
    assert 'output_dir:=logs with spaces' in args
    assert not (workspace / 'maps').exists()
    assert not pcd.with_suffix('.yaml').exists()  # Generated later by the localizer.


def test_last_custom_overrides_win_and_only_selected_paths_are_checked(launcher):
    workspace, run, _ = launcher
    pcd = make_map(workspace / 'selected map/custom.pcd')
    yaml = make_map(workspace / 'selected grid/grid.yaml')
    result, args = run('map_pcd:=missing.pcd', 'map_yaml:=missing.yaml',
                       f'map_pcd:={pcd}', f'map_yaml:={yaml}')
    assert result.returncode == 0, result.stderr
    assert [a for a in args if a.startswith('map_pcd:=')] == [f'map_pcd:={pcd}']
    assert [a for a in args if a.startswith('map_yaml:=')] == [f'map_yaml:={yaml}']


def test_yaml_only_selects_matching_pcd(launcher):
    workspace, run, _ = launcher
    pcd = make_map(workspace / 'custom/scene.pcd')
    yaml = make_map(pcd.with_suffix('.yaml'))
    make_map(workspace / 'maps/map.pcd')
    result, args = run(f'map_yaml:={yaml}')
    assert result.returncode == 0, result.stderr
    assert f'map_pcd:={pcd}' in args
    assert f'map_yaml:={yaml}' in args


@pytest.mark.parametrize('missing', ['pcd', 'yaml'])
def test_missing_explicit_map_is_rejected_even_when_defaults_exist(launcher, missing):
    workspace, run, _ = launcher
    pcd = make_map(workspace / 'maps/map.pcd')
    make_map(pcd.with_suffix('.yaml'))
    options = [f'map_pcd:={pcd}', f'map_{missing}:=missing.{missing}']
    result, args = run(*options)
    assert result.returncode != 0
    assert args is None
    assert f'missing.{missing}' in result.stderr
    assert not (workspace / 'artifacts').exists()


def test_missing_default_does_not_copy_arbitrary_artifact(launcher):
    workspace, run, _ = launcher
    artifact = make_map(workspace / 'artifacts/unverified/map.pcd')
    result, args = run()
    assert result.returncode != 0
    assert args is None
    assert artifact.exists()
    assert not (workspace / 'maps').exists()
    assert 'map_pcd:=' in result.stderr


@pytest.mark.parametrize('flag', ['--show-args', '--help'])
def test_launch_help_does_not_require_map_or_create_run_directory(launcher, flag):
    workspace, run, mode = launcher
    result, args = run(flag)
    assert result.returncode == 0, result.stderr
    assert args == ['launch', 'nav_stack', f'{mode}.launch.py', flag]
    assert not (workspace / 'artifacts').exists()


@pytest.mark.parametrize('option', ['map_pcd:=', 'map_yaml:='])
def test_empty_explicit_map_path_is_rejected(launcher, option):
    _, run, _ = launcher
    result, args = run(option)
    assert result.returncode != 0
    assert args is None
