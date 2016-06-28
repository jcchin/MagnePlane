import math, numpy, scipy
from openmdao.core.component import Component
from openmdao.api import IndepVarComp, Component, Problem, Group, ScipyOptimizer, ExecComp, SqliteRecorder


class system_total_weight(Component):
    def __init__(self):
        super(system_total_weight,self).__init__()

        self.add_output('pylon_weight', val=0.0, desc='Total weight of pylons ', unit='kg')
        self.add_output('tube_weight', val=0.0, desc='Total weight of tube ', unit='kg')
        self.add_output('pod_total_weight', val=0.0, desc='Total weight of tube', unit='kg')
        self.add_output('sub_track_weight', val=0.0, desc='Total weight of subtrack ', unit='kg')

    def solve_nonlinear(self, params, unknowns, resids):


def pylon_weight(self, nu, pylon_den, vol):
    t_weight = nu * pylon_den * vol
    return t_weight


def tube_weight(self, tube_thickness, route_len, tube_den):
    t_weight = tube_thickness * route_len * tube_den
    return t_weight


def sub_track(self, track_vol, route_len):
    t_weight = track_vol * route_len
