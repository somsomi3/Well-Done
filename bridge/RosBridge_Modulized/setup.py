from setuptools import setup, find_packages

setup(
    name="robot_bridge",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "flask",
        "requests",
    ],
    entry_points={
        'console_scripts': [
            'robot_bridge=robot_bridge.main:main',
        ],
    },
)