from setuptools import setup, find_packages

package_name = 'multi_purpose_mpc_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='takao',
    maintainer_email='takao@example.com',
    description='MPC Controller Package',
    license='Apache License 2.0',
)
