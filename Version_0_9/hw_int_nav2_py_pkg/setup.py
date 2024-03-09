from setuptools import find_packages, setup

package_name = 'hw_int_nav2_py_pkg'

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
    maintainer='fkung',
    maintainer_email='fkung@todo.todo',
    description='Manages serial communication with external hardware or robot controller',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "serial_com = hw_int_nav2_py_pkg.serial_com_node:main",
            "rcstate_pub = hw_int_nav2_py_pkg.rcstate_pub_node:main",
            "teleop_key = hw_int_nav2_py_pkg.teleop_key:main"
        ],
    },
)
