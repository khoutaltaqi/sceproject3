from setuptools import find_packages, setup

commands = ['check', 'arm', 'calibrate', 'trial', 'measure', 'plot']
setup(
    name='rm_phase3', version='1.1.0', packages=find_packages(exclude=['tests']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/rm_phase3']),
                ('share/rm_phase3', ['package.xml', 'config.json'])],
    install_requires=['setuptools'],
    maintainer='Khoutal Taqi', maintainer_email='taqi.khoutal@gmail.com',
    description='Vertical AprilTag approach with odometry and camera correction',
    license='Apache-2.0',
    entry_points={'console_scripts': [f'{s} = rm_phase3.{s}:main' for s in commands]},
)
