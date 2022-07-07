from setuptools import setup

package_name = 'graph_generator_package'
main_package = 'graph_generator_package/mainPackage'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, main_package],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lucacordioli',
    maintainer_email='luca.cordioli2000@gmail.com',
    description='Generate graph for current map.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'graph_generator_node = graph_generator_package.graph_generator_node:main'
        ],
    },
)
