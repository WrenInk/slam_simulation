from glob import glob
from pathlib import Path
from setuptools import setup
setup(name='nav_stack', version='0.1.0', packages=['nav_stack'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/nav_stack']),
                  ('share/nav_stack', ['package.xml']),
                  ('share/nav_stack/launch', glob('launch/*.launch.py')),
                  ('share/nav_stack/config', [p for p in glob('config/*') if Path(p).is_file()]),
                  ('share/nav_stack/urdf', glob('urdf/*.urdf')),
                  ('share/nav_stack/meshes', glob('meshes/*')),
                  ('share/nav_stack/scan_patterns', glob('scan_patterns/*')),
                  ('share/nav_stack/worlds', glob('worlds/*.xml'))],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': [
          'backend = nav_stack.backend_node:main',
          'fastlio_map = nav_stack.fastlio_map_node:main',
          'bridge = nav_stack.bridge_node:main',
          'localizer = nav_stack.localizer_node:main',
          'simulator = nav_stack.sim_node:main']})
