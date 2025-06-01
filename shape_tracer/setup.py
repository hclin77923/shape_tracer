from setuptools import setup

package_name = 'shape_tracer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/shape_tracer.launch.py']),
        ('share/' + package_name + '/config', ['config/shapes.json']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Trace 2D shapes in 3D space using xArm7 and MoveIt in ROS2',
    license='TODO:',
    entry_points={
        'console_scripts': [
            'shape_tracer_node = shape_tracer.shape_tracer_node:main'
        ],
    },
)