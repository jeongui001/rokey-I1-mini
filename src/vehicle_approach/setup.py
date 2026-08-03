from setuptools import find_packages, setup

package_name = 'vehicle_approach'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hwangjeongui',
    maintainer_email='hwangjeongui01@gmail.com',
    description='웹캠이 발행한 차량 위치로 접근하고 오크디 depth로 전방 정지를 판단하는 노드',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'vehicle_approach_node = vehicle_approach.vehicle_approach_node:main',
        ],
    },
)
