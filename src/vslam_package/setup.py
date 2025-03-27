from setuptools import setup

package_name = 'vslam_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=['vslam_package.aruco_detector'],
    install_requires=['setuptools', 'opencv-python', 'cv-bridge'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'aruco_detector = vslam_package.aruco_detector:main',  # Make sure this matches the path and method
        ],
    },
)

