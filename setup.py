from setuptools import setup, find_packages

setup(name='hyperloop',
	version = '2.0',
	packages=find_packages('src'),
	package_dir={'': 'src'})

# not a true package, currently only packages Python code
