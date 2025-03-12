from setuptools import setup, find_packages

setup(
    name='beavr',
    version='1.0.0',
    packages=find_packages(where='beavr/src'),
    package_dir={'': 'beavr/src'},
    description='BeaVR-Bot: Bimanual, multi-Embodiment, Accessible, Virtual Reality Teleoperation System for Robots',
    install_requires=[
        # Add your dependencies here
    ],
    package_data={
        'components.environment': ['assets/**/*', 'configs/*'],
        'robot': ['assets/*', 'configs/*'],
        'robot.allegro.configs': ['*.yaml'],
        'components.operators.configs': ['*.yaml'],
    },
    include_package_data=True,
    python_requires='>=3.6',
)
