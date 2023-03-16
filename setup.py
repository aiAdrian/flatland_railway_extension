# python setup.py sdist
# pip install twine
# python setup.py bdist_wheel
# python setup.py sdist
# twine upload dist/*

from setuptools import find_packages
from setuptools import setup

setup(
    name='flatland-railway-extension',
    version='0.2.1',
    author='Adrian Egli',
    author_email="3dhelp@gmail.com",
    description='Extends Flatland Railway Simulator with helpful features.',
    url='https://github.com/aiAdrian/flatland_railway_extension',
    keywords='flatland, railway, extension, dynamics, simulation, multi-agent, reinforcement learning',
    python_requires='>=3.6, <4',
    packages=find_packages('.'),
    install_requires=[
        'numpy',
        'matplotlib==3.2.2',
        'flatland-rl-optimised-code'
    ],
)