from setuptools import setup
from setuptools import find_packages

setup(
    name='Drone Simulator',
    version='1.0.0',
    description='User interface for 6DoF simulation of thrust-vectored monocopter drone',
    author='Mike Timmerman',
    author_email='timmermanmike@hotmail.com',
    url='https://github.com/...',

    install_requires=['numpy',
                      'PyQt5',
                      'matplotlib',
                      'pandas',
                      'pyrr',
                      'PyOpenGL',
                      'pywin32-ctypes'],

    packages=find_packages(),
    entry_points={
        'console_scripts': [
            'Simulator = main:main',
        ],
    },
)
