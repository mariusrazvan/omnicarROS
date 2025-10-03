from setuptools import find_packages, setup

package_name = 'read_elrs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pisix',
    maintainer_email='pisix@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "read_remote = read_elrs.read_remote:main",
            "control_relays = read_elrs.control_relays:main",
            "run_motors = read_elrs.run_motors:main",
            "control_lights = read_elrs.control_lights:main",
            "proximity_sense = read_elrs.proximity_sense:main",
            "control_beep = read_elrs.control_beep:main",
            "read_mpu = read_elrs.read_mpu:main",
            "read_front_distance = read_elrs.read_front_distance:main",
            "transform_cmd_vel = read_elrs.transform_cmd_vel:main"
        ],
    },
)
