from setuptools import find_packages, setup

package_name = 'tag_follower_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           'tag_follower = tag_follower_pkg.tag_follower:main',       
 ],
    },
)


# from setuptools import setup
# from glob import glob

# package_name = 'ball_tracker'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=[package_name],
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         ('share/' + package_name + '/launch', glob('launch/*launch.py')),
#         ('share/' + package_name + '/config', glob('config/*.yaml')),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='newans',
#     maintainer_email='josh.newans@gmail.com',
#     description='TODO: Package description',
#     license='TODO: License declaration',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#             'detect_ball = ball_tracker.detect_ball:main',
#             'detect_ball_3d = ball_tracker.detect_ball_3d:main',
#             'follow_ball = ball_tracker.follow_ball:main',
#         ],
#     },
# )