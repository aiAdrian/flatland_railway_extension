import os

from setuptools import find_packages
from setuptools import setup


def read_text(file_name: str):
    return open(os.path.join('.', file_name)).read()

with open('README.md', 'r', encoding='utf8') as readme_file:
    readme = readme_file.read()


setup(
    name='flatland-railway-extension',
    version='0.1.0',
    author='Adrian Egli',
    author_email = "3dhelp@gmail.com",
    description='Extends Flatland Railway Simulator with helpful features.',
    long_description=readme,
    long_description_content_type="text/markdown",
    url='https://github.com/aiAdrian/flatland_railway_extension',
    keywords='flatland, railway, extension, dynamics, simulation, multi-agent, reinforcement learning',
    python_requires='>=3.6, <4',
    packages=find_packages('.'),
    license=read_text("LICENSE"),
    install_requires=[
        'flatland-rl>=3.0.15',
        'numpy',
        'matplotlib',
    ]
)
