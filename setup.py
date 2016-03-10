from setuptools import setup

setup(name='Distutils',
    version='0.0',
    description='HSoE mapping hexacopter software',
    packages=['hsoe_copter'],
	install_requires=['pymavlink','pyserial']
    )
