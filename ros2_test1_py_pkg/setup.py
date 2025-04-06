from setuptools import find_packages, setup

package_name = 'ros2_test1_py_pkg'

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
    maintainer='fk',
    maintainer_email='fk@todo.todo',
    description='Basic python package, just create a node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "My_Node1 = ros2_test1_py_pkg.my_node1:main"
        ],
    },
)
