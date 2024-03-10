from setuptools import setup

package_name = 'wall_follower_alex'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar_alex',
    maintainer_email='gonzaya2003a@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "example=wall_follower.example:main",
            "safety_example=wall_follower.safety:main"
            "wall_follower_alex=wall_follower_alex.wall_follower:main"
        ],
    },
)
