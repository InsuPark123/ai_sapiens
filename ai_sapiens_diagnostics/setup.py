# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0.
from glob import glob

from setuptools import setup

setup(
    name='ai_sapiens_diagnostics', version='0.1.0',
    packages=['ai_sapiens_diagnostics'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ai_sapiens_diagnostics']),
        ('share/ai_sapiens_diagnostics', ['package.xml', 'README.md']),
        ('share/ai_sapiens_diagnostics/config', glob('config/*.yaml')),
        ('share/ai_sapiens_diagnostics/launch', glob('launch/*.launch.py')),
        ('share/ai_sapiens_diagnostics/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Wonho Yun', maintainer_email='ywh@robotis.com',
    description='Read-only robot health aggregation and app telemetry',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'health_node = ai_sapiens_diagnostics.node:main',
        'hat_collector = ai_sapiens_diagnostics.hat_node:main',
        'hat_collect_once = ai_sapiens_diagnostics.hat_collector:main',
    ]},
)
