import glob
import os
from setuptools import setup

package_name = 'wall_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/'+package_name, ['package.xml', "wall_follower/params.yaml", "wall_follower/params_sim.yaml"]),
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/wall_follower/launch', glob.glob(os.path.join('launch', '*launch.xml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "example=wall_follower.example:main",
            "safety_example=wall_follower.safety:main",
            "wall_follower=wall_follower.wall_follower:main",
            "safety_controller=wall_follower.safety_controller:main"
        ],
    },
)
