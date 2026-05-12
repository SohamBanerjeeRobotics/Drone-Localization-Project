from setuptools import find_packages, setup

package_name = 'rl_targets'

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
    maintainer='tejash',
    maintainer_email='tejash@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_publisher = rl_targets.target_publisher:main',
            'target_greedy = rl_targets.target_greedy:main'
        ],

    },
)
