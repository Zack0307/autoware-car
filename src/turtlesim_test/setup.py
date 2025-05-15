from setuptools import setup, find_packages

package_name = 'turtlesim_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),  # 自動包含所有包
    package_data={
            'turtle_sim' : ['Rosmaster_Lib/*'],     
    } ,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dllab',
    maintainer_email='dllab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ps4_turtle = turtlesim_test.joy_to_twist:main',
            'ps4 = turtlesim_test.ps4:main'
        ],
    },
)
