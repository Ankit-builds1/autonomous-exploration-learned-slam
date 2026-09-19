from setuptools import find_packages, setup

package_name = 'explore_nav'

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
    maintainer='ommdash',
    maintainer_email='ommdash@todo.todo',
    description='Frontier-based autonomous exploration for SLAM+Nav2',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'frontier_explorer = explore_nav.frontier_explorer:main',
            'llm_task_planner = explore_nav.llm_task_planner:main',
            'task_executor = explore_nav.task_executor:main',
        ],
    },
)
