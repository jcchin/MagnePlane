from setuptools import setup, find_packages

setup(name='hyperloop',
	version = '2.0',
	author = 'NASA MARTI'
	packages = find_packages('src'),
	package_dir = {'': 'src'},
	install_requires = [pycycle, openmdao, scipy, matplotlib, numpy],
	dependency_links = ['https://github.com/JustinSGray/pyCycle.git'],
	tests_require = ['pytest'])
