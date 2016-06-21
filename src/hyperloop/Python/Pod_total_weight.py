import math, numpy, scipy
from openmdao.core.component import Component
from openmdao.api import IndepVarComp, Component, Problem, Group, ScipyOptimizer, ExecComp, SqliteRecorder


class Pod_total_weight(Component):
    def __init__(self):
        '''input params '''
        super(Pod_total_weight,self).__init__()



        #Payload System Weight:
        self.add_param('safety', val=0.0, desc='Total Weight of Safety', units='kg')
        self.add_param('battery', val=0.0, desc='Total Weight of Battery System', units='kg')
        self.add_param('interior', val=0.0, desc='Total weight of the interior frame', units='kg')
        self.add_param('baggage_allowance', val=0.0, desc='Total Weight of Baggage', units='kg')
        self.add_param('passengers', val=0.0, desc='Total Weight of passengers', units='kg')

        #Suspension System Weight:
        self.add_param('halbach_array_magnets', val = 0.0, desc = 'Total weight of magnets for suspension')
        self.add_param('lev_mounts', val=0.0, desc='Weight of the mechanical mounting system')


        #Propulsion System: Will have design switch to switch between LSM and LIM model

        self.add_params('compressor_weight', val=0.0, desc='Total weight of Compressor', unit='kg')
        #LSM
        self.add_param('perm_magnets', val=0.0, desc='', units = 'kg')

        #LIM
        self.add_param('conductor_plates', val=0.0, desc='', units = 'kg')


        #Power Electronics
        self.add_param('battery_weight', val=0.0, desc='Total Weight of Battery System', units='kg')
        self.add_param('motor', val=0.0, desc='Weight for the electric motor', units='kg')
        self.add_param('PMAD', val=0.0, desc='Weight for power management and distribution', units='kg')

       #Structure:
        self.add_param('exterior_shell', val=0.0, desc='Total weight of Shell structure', unit='kg')

    def solve_nonlinear(self, params, unknowns, resids):
        # Payload System Weight:
        safety = params['safety']
        battery =  params['battery']
        interior = params['interior']
        baggage_allowance = params['baggage_allowance']
        passengers = params['passengers']

        # Suspension System Weight:
        halbach_array_magnets = params['halbach_array_magnets']
        lev_mounts = params['lev_mounts']

        # Propulsion System: Will have design switch to switch between LSM and LIM model
        compressor_weight = params['compressor_weight']

        # LSM
        perm_magnets = params['perm_magnets']

        # LIM
        conductor_plates = params['conductor_plates']

        # Power Electronics
        battery_weight = params['battery_weight']
        motor = params['motor']
        PMAD = params['PMAD']

        # Structure:
        exterior_shell = params['exterior_shell']


    def suspension_weight(self, length_pod,mag_weight_unit, alpha, beta, mount_weight_unit,design):

        def LSM(self, )







    def propulsion(self, length_pod, mag_weight_unit, alpha, conductor_plates, ):



    def power_electronics(self, length_pod*mag_weight * alpha):


    def structure(self, length_pod*mag_weight * alpha):