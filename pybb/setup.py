from setuptools import setup, find_packages

setup(name='black_box',
      version='1.0.0',
      description='A library for interacting with a robotic black box',
      url='https://github.com/ropod-project/black-box',
      author='Alex Mitrevski',
      author_email='aleksandar.mitrevski@h-brs.de',
      keywords='black_box fault_detection robotics',
      packages=find_packages(exclude=['contrib', 'docs', 'tests']),
      project_urls={
          'Source': 'https://github.com/ropod-project/black-box'
      })
