from setuptools import setup

package_name = 'scan_matching'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saijay0408',
    maintainer_email='sai2001jayanth@gmail.com',
    description='scan matching node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_matcher = scan_matching.scan_matcher:main',
        ],
    },
)
