from setuptools import find_packages, setup

package_name = 'trajectory_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='f.iotti@studenti.unipi.it',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_node = trajectory_planner.pose_node:main',
            'twist_node = trajectory_planner.twist_node:main'
        ],
    },
)
