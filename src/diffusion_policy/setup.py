from setuptools import find_packages, setup

package_name = 'diffusion_policy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='starsky',
    maintainer_email='starsky@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'eval_real_robot = diffusion_policy.eval_real_robot:main',
            'diffusion_policy_test = diffusion_policy.diffusion_policy_test:main',
        ],
    },
)
