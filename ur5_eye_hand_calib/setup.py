from setuptools import setup

package_name = 'ur5_eye_hand_calib'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/calibration', ['ur5_eye_hand_calib/camera_calibration.yaml']),
        ('share/' + package_name + '/config', ['ur5_eye_hand_calib/config/fixed_board_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/fixed_board_calibration.launch.py']),
    ],
    install_requires=['setuptools', 'tf-transformations', 'ament-index-python'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='UR5 Eye-to-Hand Calibration Package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'eye_to_hand_calibrator = ur5_eye_hand_calib.calibration_node:main',
            'fixed_board_calibrator = ur5_eye_hand_calib.fixed_board_calibrator:main',
            'interactive_calibrator = ur5_eye_hand_calib.interactive_calibrator:main',
            'click_to_verify = ur5_eye_hand_calib.click_to_verify:main',
            'pixel_to_tcp = ur5_eye_hand_calib.pixel_to_tcp_node:main',
            'overlay_visualizer = ur5_eye_hand_calib.overlay_visualizer:main',
        ],
    },
)
