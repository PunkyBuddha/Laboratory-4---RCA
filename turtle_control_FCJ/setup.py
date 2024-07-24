from setuptools import setup

package_name = 'turtle_control_FCJ'

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
    maintainer='Your Name',
    maintainer_email='your_email@domain.com',
    description='Turtle control package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_control = turtle_control_FCJ.turtle_control:main',
            'move_action_server = turtle_control_FCJ.move_action_server:main',
            'move_action_client = turtle_control_FCJ.move_action_client:main',
        ],
    },
)
