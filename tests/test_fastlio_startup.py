"""Exercise parameter validation in the built FAST-LIO executable."""
import os
from pathlib import Path
import signal
import subprocess
import threading

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
BINARY = ROOT / 'install/fast_lio/lib/fast_lio/fastlio_mapping'
pytestmark = pytest.mark.skipif(not BINARY.is_file(), reason='Build FAST-LIO first')


def node_environment(tmp_path):
    env = os.environ.copy()
    env.update(ROS_LOG_DIR=str(tmp_path / 'ros_logs'), ROS_DOMAIN_ID='119',
               ROS_LOCALHOST_ONLY='1')
    return env


@pytest.mark.parametrize('translation,rotation,error', [
    (None, None, 'mapping.extrinsic_T'),
    ([0.0, 0.0], list([1.0, 0.0, 0.0] * 3), 'mapping.extrinsic_T'),
    ([0.0] * 3, [1.0] * 8, 'mapping.extrinsic_R'),
    ([0.0, float('nan'), 0.0], [1.0] * 9, 'mapping.extrinsic_T'),
    ([0.0] * 3, [float('inf')] * 9, 'mapping.extrinsic_R'),
])
def test_invalid_extrinsics_exit_with_parameter_error(tmp_path, translation, rotation, error):
    command = [str(BINARY)]
    if translation is not None:
        params = tmp_path / 'parameters.yaml'
        params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
            'mapping.extrinsic_T': translation, 'mapping.extrinsic_R': rotation,
        }}}))
        command += ['--ros-args', '--params-file', str(params)]
    result = subprocess.run(command, env=node_environment(tmp_path),
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, timeout=15)
    assert result.returncode == 1, result.stdout
    assert error in result.stdout, result.stdout
    assert 'finite values' in result.stdout, result.stdout


def test_simulation_parameters_start_and_shutdown_cleanly(tmp_path):
    command = [str(BINARY), '--ros-args', '--params-file',
               str(ROOT / 'src/nav_stack/config/simulation.yaml')]
    process = subprocess.Popen(command, env=node_environment(tmp_path),
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True)
    ready = threading.Event()
    output = []

    def read_output():
        for line in process.stdout:
            output.append(line)
            if 'Node init finished.' in line:
                ready.set()

    reader = threading.Thread(target=read_output, daemon=True)
    reader.start()
    try:
        assert ready.wait(15), ''.join(output)
        assert process.poll() is None, ''.join(output)
        process.send_signal(signal.SIGINT)
        assert process.wait(timeout=10) == 0, ''.join(output)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5)
        reader.join(timeout=5)
        process.stdout.close()
