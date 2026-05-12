from setuptools import find_packages, setup

package_name = 'pyrosim_temp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/data', ['data/' + 'pyrosim_data.csv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soham-banerjee',
    maintainer_email='soham-banerjee@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'temp_sender = pyrosim_temp.temp_sender:main'
        ],
    },
)
