from setuptools import setup, find_packages

package_name = 'ros_tkinter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),  #[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cht',
    maintainer_email='gabbyru2@gmail.com',
    description='ROS2 node that turns messages into a GUI to visualize ROS topics',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ros_tkinter_node = ros_tkinter.ros_tkinter:main"
        ],
    },
)
