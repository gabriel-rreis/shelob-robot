from setuptools import setup

package_name = 'hexapod_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robo',
    maintainer_email='43526807+gabriel-rreis@users.noreply.github.com',
    description='3-DOF hexapod robot control package',
    license='GNU General Public License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_driver = hexapod_controller.servo_driver:main'
        ],
    },
)
