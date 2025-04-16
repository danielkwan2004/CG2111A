from setuptools import setup, find_packages
setup(
    name='slam',
    version='0.1.0',
    packages=find_packages(include=[ 'slam.slam.*']),
    author='EPP2 Teaching Team',
    python_requires='>=3.6',
    install_requires=[
        'numpy',
        'matplotlib',
        'pyserial'
    ],
    description='EPP2 slam Library',
)
