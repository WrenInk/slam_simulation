"""Start an isolated headless arena and verify four Nav2 goals."""
from pathlib import Path
from datetime import datetime
import os
import signal
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
output = root/'artifacts'/('acceptance_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
output.mkdir(parents=True)
print('Validation output: '+str(output), flush=True)
with (output/'launch.log').open('w') as log:
    process = subprocess.Popen(['ros2', 'launch', 'nav_stack', 'omni_navigation.launch.py',
        'gui:=false', 'rviz:=false', 'output_dir:='+str(output)],
        cwd=root, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        result = subprocess.run([sys.executable, str(root/'scripts/audit_omni_navigation.py'),
            '--output', str(output)], cwd=root, timeout=750)
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=35)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)
raise SystemExit(result.returncode)
