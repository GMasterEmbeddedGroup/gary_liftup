from setuptools import setup, find_packages

package_name = 'gary_liftup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gmaster',
    maintainer_email='juntong_2021@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "start_up_down_node = gary_liftup:start_up_down_node",
            "start_stretch_arm_node = gary_liftup:start_stretch_arm_node",
        ],
    },
)
