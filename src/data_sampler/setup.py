from setuptools import find_packages, setup

package_name = 'data_sampler'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','h5py'],
    zip_safe=True,
    maintainer='starsky',
    maintainer_email='starsky@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_sampler = data_sampler.data_sampler:main',
        ],
    },
)
