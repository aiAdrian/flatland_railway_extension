# python setup.py sdist
# pip install twine
# twine upload dist/*

import os

from setuptools import find_packages
from setuptools import setup


setup(
    name='flatland-railway-extension',
    version='0.1.2',
    author='Adrian Egli',
    author_email = "3dhelp@gmail.com",
    description='Extends Flatland Railway Simulator with helpful features.',
    url='https://github.com/aiAdrian/flatland_railway_extension',
    keywords='flatland, railway, extension, dynamics, simulation, multi-agent, reinforcement learning',
    python_requires='>=3.6, <4',
    packages=find_packages('.'),
    install_requires=[
        'flatland-rl>=3.0.15',
        'numpy',
        'matplotlib',
    ]
)
