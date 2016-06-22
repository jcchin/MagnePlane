from setuptools import setup, find_packages

setup(name='hyperloop',
	version = '2.0',
	author = 'NASA MARTI',
	packages = find_packages('src'),
	package_dir = {'': 'src'},
	dependency_links = ['https://github.com/JustinSGray/pyCycle.git'],
	install_requires = [openmdao, scipy, matplotlib, numpy],
	tests_require = ['pytest'])
