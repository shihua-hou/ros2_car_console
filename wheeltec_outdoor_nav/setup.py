from setuptools import setup

package_name = 'wheeltec_outdoor_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Claude',
    maintainer_email='claude@anthropic.com',
    description='RTK-assisted outdoor navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
)
