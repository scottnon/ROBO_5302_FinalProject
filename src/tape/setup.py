from setuptools import find_packages, setup

package_name = 'tape'

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
    maintainer='kingpooper',
    maintainer_email='kingpooper@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'image_subscriber = tape.image_subscriber:main',
        'tape_follower = tape.tape_follower:main',
        'my_tape_follower = tape.my_tape_follower:main',
        'my_sexy_tape_follower = tape.my_sexy_tape_follower:main',
        'my_blue_tape_follower = tape.my_blue_tape_follower:main',
        'tom_blue_tape = tape.tom_blue_tape:main',
        'james_blue_tape = tape.james_blue_tape:main',
        'stop_sign = tape.stop_sign:main',
        'school_zone = tape.school_zone:main',
        'obs_avoid = tape.obs_avoid:main',
        ],
    },
)
