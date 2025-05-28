from setuptools import find_packages, setup

package_name = 'cv_mono_ir'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='paz',
    maintainer_email='niccolo.pasetto@studenti.unipd.it',
    description='Publisher for the C.V. algorithm on image detection for mono vision',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = cv_mono_ir.publisher:main'
        ],
    },
)
