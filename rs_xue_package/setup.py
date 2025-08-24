from setuptools import setup, find_packages
import os
import glob

so_files = glob.glob('rs_xue/*.so')

setup(
    name='rs_xue',
    version='1.0.0',
    description='RealSense LiDAR Python Interface By Xue',
    author='Aoru.Xue',
    author_email='aoru45@shanghaitech.edu.cn',
    packages=find_packages(),
    package_data={
        'rs_xue': ['*.so'],
    },
    include_package_data=True,
    python_requires='>=3.8',
    install_requires=[
        'numpy',
    ],
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
        'Programming Language :: Python :: 3.12',
    ],
    zip_safe=False,
)