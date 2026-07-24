from setuptools import setup

package_name = 'wheeltec_webapp'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    package_data={package_name: ['static/*', 'static/voice/*']},
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    description='小车 Web 控制台: 浏览器遥控/建图/存图/导航, 轻量级 web 版 rviz',
    license='MIT',
    entry_points={
        'console_scripts': [
            'web = wheeltec_webapp.app:main',
        ],
    },
)
