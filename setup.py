from setuptools import setup, find_packages

setup(name='Robot_Haptics UofC',
      version='0.0.1',
      author='Huanjun Zhao',
      author_email='huanjun.zhao@ucalgary.ca',
      long_description=open('README.md').read(),
      long_description_content_type="text/markdown",
      license="BSD-3-Clause",
      packages=find_packages(include=['unitree_sdk2py','unitree_sdk2py.*']),
      description='Unitree robot sdk version 2 for python, modified by Huanjun Zhao, University of Calgary. Copyright (c) 2016-2024 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics") All rights reserved.',
      project_urls={
            "Source Code": "https://github.com/HuanjunZhao/unitree_sdk2_python",
      },
      python_requires='>=3.8',
      install_requires=[
            "cyclonedds==0.10.2",
            "numpy",
            "opencv-python",
      ],
      )