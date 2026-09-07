from setuptools import setup
setup(name='fwrl_ros', version='0.1.0', packages=['fwrl_ros'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/fwrl_ros']),
                  ('share/fwrl_ros', ['package.xml']),
                  ('share/fwrl_ros/launch', ['launch/simulation.launch.py'])],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': ['simulator = fwrl_ros.nodes:simulator_main',
                                       'controller = fwrl_ros.nodes:controller_main',
                                       'analyzer = fwrl_ros.nodes:analyzer_main']})
