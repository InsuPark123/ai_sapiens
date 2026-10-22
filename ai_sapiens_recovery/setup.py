from setuptools import setup
from glob import glob
setup(name='ai_sapiens_recovery', version='0.1.0', packages=['ai_sapiens_recovery'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/ai_sapiens_recovery']),
                  ('share/ai_sapiens_recovery', ['package.xml', 'README.md']),
                  ('share/ai_sapiens_recovery/config', ['config/k1.yaml']),
                  ('share/ai_sapiens_recovery/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Wonho Yun', maintainer_email='ywh@robotis.com', license='Apache-2.0',
      description='Opt-in K1 recovery and automatic rearming into damping',
      entry_points={'console_scripts': ['recovery_node = ai_sapiens_recovery.node:main']})
