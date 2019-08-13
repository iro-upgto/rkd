from setuptools import setup

# Current status: pre-alpha

setup(name='rkd',
      version='0.1.0',
      description='Python library for kinematic analysis of robots',
      author='IRO-UPGTO',
      author_email='jdelossantos@upgto.edu.mx',
      license = "MIT",
      keywords=["Robotics","Kinematics"],
      url='https://github.com/iro-upgto/rkd',
      packages=['rkd'],
      install_requires=['numpy','sympy','matplotlib','scipy'],
      classifiers=[
      "Development Status :: 2 - Pre-Alpha",
      "Intended Audience :: Education",
      "Intended Audience :: Developers",
      "License :: OSI Approved :: MIT License",
      "Operating System :: OS Independent",
      "Programming Language :: Python",
      "Programming Language :: Python :: 3.6",
      "Programming Language :: Python :: Implementation :: CPython",
      ]
      )
