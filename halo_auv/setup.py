from setuptools import setup

package_name = 'halo_auv'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                    'launch/auv.launch.xml',
                                    'config/auv_cam_params.yaml',
                                    'config/tag.yaml',
                                    'config/video.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oubre',
    maintainer_email='oubrejames@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = halo_auv.camera_stream:main',
            'auv_control = halo_auv.control_auv:main',
            'auv_node = halo_auv.halo_auv:main',],
    },
)
