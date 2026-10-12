from glob import glob
from setuptools import setup

package_name = 'greeting_voice'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='voice-owner-B',
    maintainer_email='dev@x-humanoid-cloud.com',
    description='天轶 2.5 迎宾语音包：平台简单动作模式按键语音 + 编排层 /greeting/speak 语音服务端',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'speak_action_server = greeting_voice.speak_action_server:main',
            'simple_action_voice_node = greeting_voice.simple_action_voice_node:main',
        ],
    },
)
