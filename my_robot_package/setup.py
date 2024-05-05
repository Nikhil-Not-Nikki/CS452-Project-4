from setuptools import find_packages, setup

package_name = 'my_package'

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
    maintainer='nikkic2302',
    maintainer_email='nikkic2302@gmail.com',
    description='Task 4 Project 1',
    license='Apache license 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	    'talker = my_package.publisher_member_function:main',
	    'listener = my_package.subscriber_member_function:main',
	    'identifier = my_package.environment_identifier:main',
	],

    },
)
