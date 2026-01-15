from setuptools import find_packages, setup

package_name = 'leros2_record_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/utils.launch.py']),
        ('share/' + package_name + '/config', ['config/record_utils.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tamer',
    maintainer_email='nicolas@gres.io',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'leros2_record_utils = leros2_record_utils.record_utils:main',
        ],
    },
)
