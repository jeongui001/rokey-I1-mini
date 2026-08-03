from setuptools import find_packages, setup

package_name = 'webcam_perception'

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
    description='웹캠 ROI 감시, 정지 판정, 픽셀→map 변환 후 차량 초기 위치 발행',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'webcam_perception_node = webcam_perception.webcam_perception_node:main',
            'homography_calibration_tool = webcam_perception.homography_calibration_tool:main',
        ],
    },
)
