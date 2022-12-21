import os

from setuptools import find_packages
from setuptools import setup


def read_text(file_name: str):
    return open(os.path.join('.', file_name)).read()


setup(
    name='flatland_railway_extension',
    version='0.1.0',
    author='Adrian Egli',
    author_email = "3dhelp@gmail.com",
    description='Extends Flatland Railway Simulator with helpful features.',
    long_description='',
    url='https://github.com/aiAdrian/flatland_railway_extension',
    keywords='flatland, railway, extension, dynamics, simulation, multi-agent, reinforcement learning',
    python_requires='>=3.6, <4',
    packages=find_packages(include=['flatland_railway_extension', 'flatland_railway_extension.*']),
    license=read_text("LICENSE"),
    install_requires=[
        'flatland-rl>=3.0.15',
        'numpy',
        'matplotlib',
    ],
    entry_points={
        'runners': [
            'demo_flatland_dynamics=demo_flatland_dynamics.py',
        ]
    }
)
