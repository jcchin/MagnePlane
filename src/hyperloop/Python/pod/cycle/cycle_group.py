"""
Group for the Cycle containing the following components:
Flowpath and Compressor Mass
"""
from __future__ import print_function
from openmdao.api import IndepVarComp, Component, Problem, Group

from hyperloop.Python.pod.cycle.flowpath import FlowPath
from hyperloop.Python.pod.cycle.compressor_mass import CompressorMass

class Cycle(Group):
	def __init__(self):
		super(TubeGroup, self).__init__()

		self.add('Flowpath', FlowPath())
		self.add('CompressorMass', CompressorMass())

		self.connect('CompressorMass.h_in', 'Flowpath.inlet.Fl_0:tot:h')
		self.connect('CompressorMass.h_out', 'Flowpath.comp.Fl_0:tot:h')
		self.connect('CompressorMass.comp_inletR', 'FlowPath.inlet.Fl_O:stat:area')

if __name__ == "__main__":
    prob = Problem()
    root = prob.root = Group()

    prob.setup()
    prob.root.list_connections()
    prob.run()