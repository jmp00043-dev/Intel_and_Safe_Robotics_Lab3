
from setuptools import setup

package_name = 'pure_pursuit'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    package_data={
        package_name: ['data.csv'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='natha',
    maintainer_email='natha@todo.todo',
    description='Pure Pursuit waypoint following for F1TENTH',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit_node = pure_pursuit.pure_pursuit_node:main',
        ],
    },
)
