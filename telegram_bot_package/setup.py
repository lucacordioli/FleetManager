from setuptools import setup

package_name = 'telegram_bot_package'
botCallbacks = 'telegram_bot_package/botCallbacks'
data = 'telegram_bot_package/data'
rosDialog = 'telegram_bot_package/rosDialog'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, botCallbacks, data, rosDialog],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lucacordioli',
    maintainer_email='luca.cordioli2000@gmail.com',
    description='Telegram bot that receive requests and public in a topic.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telegram_bot_node = telegram_bot_package.telegram_bot_node:main'
        ],
    },
)
