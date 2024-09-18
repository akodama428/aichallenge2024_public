from setuptools import find_packages, setup
import os
import glob
package_name = 'local_planner_custom'
submodules = ['local_planner_custom/GraphBasedLocalTrajectoryPlanner',
              'local_planner_custom/GraphBasedLocalTrajectoryPlanner/graph_ltpl']
setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages=[package_name] + submodules,
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob.glob('launch/*xml')),
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
            'local_planner_custom_node = local_planner_custom.local_planner_custom_node:main'
        ],
    },
)
