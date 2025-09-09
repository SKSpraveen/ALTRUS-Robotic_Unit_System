from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'altru_framework'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'pyyaml>=5.1',
        'click>=8.0',
        'jinja2>=3.0',
    ],
    zip_safe=True,
    maintainer='ALTRUS Team',
    maintainer_email='dev@altrus.ai',
    description='Developer framework for assistive robots with multimodal control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'altrus-cli = altru_framework.cli.main:cli',
            'altrus-motion-executor = altru_framework.nodes.motion_executor:main',
            'altrus-voice-intent = altru_framework.nodes.voice_intent:main',
            'altrus-gesture-intent = altru_framework.nodes.gesture_intent:main',
        ],
    },
)
