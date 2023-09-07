from setuptools import setup

package_name = 'forw_inver_kin_urdf_based'

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
    maintainer='gregorio',
    maintainer_email='ggrilli98@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [   [
                'ee_inverse_calc = forw_inver_kin_urdf_based.ee_inv_kin_finder:main',
                'distance_publisher = forw_inver_kin_urdf_based.distance_publisher:main',
                'front_camera_forward = forw_inver_kin_urdf_based.front_camera_forw_kin_finder:main',
                'top_camera_forward = forw_inver_kin_urdf_based.top_camera_forw_kin_finder:main'
        ],

        ],
    },
)
