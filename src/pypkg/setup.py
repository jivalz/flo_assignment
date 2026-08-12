from setuptools import find_packages, setup

package_name = 'pypkg'

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
    maintainer='rover',
    maintainer_email='jivaldhingra@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop = pypkg.teleop:main',
            'wp_collector = pypkg.wp_collector:main',
            'mpc_node = pypkg.mpc_node:main',
            'mppi_node1 = pypkg.mppi_node1:main',
            'mppi_final = pypkg.mppi_final:main',
            'mppi_demo = pypkg.mppi_demo:main'
        ],
    },
)
