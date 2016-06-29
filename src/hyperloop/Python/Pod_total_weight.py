import math, numpy, scipy
from openmdao.core.component import Component
from openmdao.api import IndepVarComp, Component, Problem, Group, ScipyOptimizer, ExecComp, SqliteRecorder


class total_weight(Component):
    def __init__(self):
        '''input params '''
        super(total_weight,self).__init__()



        #Payload System Weight:
        self.add_param('safety', val=0.0, desc='Total Weight of Safety', units='kg')
        self.add_param('interior', val=0.0, desc='Total weight of the interior frame', units='kg')
        self.add_param('baggage', val=0.0, desc='Total Weight of Baggage', units='kg')
        self.add_param('passengers', val=0.0, desc='Total Weight of passengers', units='kg')


        #Suspension System Weight:
        self.add_param('halbach_array_magnets', val = 0.0, desc = 'Total weight of magnets for suspension')
        self.add_param('lev_mounts', val=0.0, desc='Weight of the mechanical mounting system')


        #Propulsion System: Will have design switch to switch between LSM and LIM model.

        self.add_params('compressor', val=0.0, desc='Total weight of Compressor', units='kg')
        self.add_params('len_pod', val=0.0, desc='Total length of pod', units='m')
        self.add_params('design', val=0.0, desc='Design switch for propulsion', units='Boolean')





        #LSM
        self.add_param('perm_den', val = 0.0, desc = 'Permanent magnets density', units = 'kg/m^3')
        self.add_param('perm_vol', val = 0.0, desc = 'Permanent magnets volume', units = 'm^3')
        self.add_param('alpha', val = 0.0, desc = 'frequency of lsm mounts', units = 'no.')

       #LIM
        self.add_param('conductor_vol', val=0.0, desc='volume of conductor', units='m^3')
        self.add_param('conductor_den', val=0.0, desc='density of conductor', units='kg/m^3')
        self.add_param('beta', val=0.0, desc='conductor frequency', units='no.')



        #Power Electronics
        self.add_param('battery', val=0.0, desc='Total Weight of Battery System', units='kg')
        self.add_param('motor', val=0.0, desc='Weight for the electric motor', units='kg')
        self.add_param('PMAD', val=0.0, desc='Weight for power management and distribution', units='kg')


        # pod Structure:
        self.add_param('volume_pod', val = 0.0, desc = 'Volume of pod', units = m^3)
        self.add_param('pod_den', val = 0.0, desc = 'Density of pod material', units = 'kg' )


        # suspension weight
        self.add_param('lev_mount', val = 0.0, desc = 'Levitation mounts', units = 'kg')
        self.add_param('psi', val = 0.0, desc = 'frequency of levitation mounts', units = 'no.')
        self.add_param('halbach_units', val = 0.0, desc = 'weight of halbach array units', units = 'kg')
        self.add_param('len_pod', val = 0.0, desc = 'length of pod', units = 'm')

        # pylon weight
        self.add_param('nu', val = 0.0, desc = 'frequency of pylons', units = 'no.'
        self.add_param('pylon_den', val = 0.0, desc = 'pylon material density', units = 'kg/m^3')
        self.add_param('vol_pylon', val = 0.0, desc = 'volume of a pylon', units = 'm^3')

        # tube weight

        self.add_param('tube_thickness', val = 0.0, desc = 'Thickness of tube')
        self.add_param('route_len', val = 0.0, desc = 'route_len')
        self.add_param('tube_den', val = 0.0, desc = 'density of tube material')

        # sub track
        self.add_param('track_vol', val = 0.0, desc = 'Volume of sub-track')
        self.add_param('route_len', val = 0.0, desc = 'Length of mission route')


        #outputs
        self.add_output('t_weight_sys', val=0.0, desc='total weight of system', units='kg')
        self.add_output(' pod_t_weight', val=0.0, desc='total weight of pod', units='kg')
        self.add_output('external_weight', val=0.0, desc='total weight of external structure', units='kg')





    def solve_nonlinear(self, params, unknowns, resids):
        # Payload:
        safety = params['safety']
        interior = params['interior']
        baggage = params['baggage']
        passengers = params['passengers']


        # Suspension System Weight:
        halbach_array_magnets = params['halbach_array_magnets']
        lev_mounts = params['lev_mounts']

        # Propulsion System: Will have design switch to switch between LSM and LIM model
        compressor = params['compressor']
        len_pod  = params['len_pod']
        design = params['design']


        # LSM
        perm_den = params['perm_den']
        perm_vol = params['perm_vol']
        alpha = params['alpha']


        # LIM
        conductor_vol = params['conductor_vol']
        conductor_den = params['conductor_den']
        beta = params['beta']

        # Power Electronics
        battery = params['battery']
        motor = params['motor']
        PMAD = params['PMAD']

    # pod Structure:
        volume_pod = params['volume_pod']
        pod_den = params['pod_den']


    #suspension weight
        lev_mounts = params['lev_mounts']
        psi = params['psi']
        halbach_units = params['halbach_units']
        len_pod = params['len_pod']


    # pylon weight
        nu = params['nu']
        pylon_den = params['pylon_den']
        vol_pylon = params['vol_pylon']


    #tube weight

        tube_thickness = params['tube_thickness']
        route_len = params['route_len']
        tube_den = params['tube_den']


    # sub track
        track_vol = params['track_vol']
        route_len = params['route_len']





        pod_t_weight = self.pod_t_weight(len_pod,perm_vol,alpha, beta,perm_den,conductor_vol,conductor_den,mount_weight_unit,design,compressor,lev_mounts, gamma,psi,  halbach_units, battery, motor, p_mad, passenger, baggage, interior, safety, volume_pod, pod_den)
        external_weight = self.external_weight(nu, pylon_den,vol_pylon )








    def pod_t_weight(self,len_pod,perm_vol,alpha, beta,perm_den,conductor_vol,conductor_den,lev_mounts,design,compressor, gamma,psi,  halbach_units, battery, motor, p_mad, passenger, baggage, interior, safety, volume_pod, pod_den):


        propulsion_weight = self.propulsion_weight(len_pod,perm_vol, perm_den,conductor_vol, conductor_den, alpha, beta,design, compressor)
        suspension_weight = self.suspension_weight(lev_mounts, gamma,psi,  halbach_units, len_pod)

        power_electronics = self.power_electronics(battery, motor, p_mad)
        payload = self.payload(passenger, baggage, interior, safety)
        exterior_shell = self.exterior_shell(volume_pod, pod_den)

        pod_t_weight = propulsion_weight + suspension_weight + power_electronics + payload + exterior_shell
        return pod_t_weight



        def propulsion_weight(self, len_pod,perm_vol, perm_den,conductor_vol, conductor_den, alpha, beta,design, compressor):

            if design == "LIM":

                LIM_weight = self.LIM(perm_vol, alpha, perm_den,len_pod, compressor )
                mount_weight = len_pod*alpha*mount_weight_unit
                t_weight = LIM_weight + mount_weight + compressor

            elif design == "LSM":

                LSM_weight = self.LSM(conductor_vol,beta,conductor_den,len_pod, compressor)
                mount_weight = len_pod * beta * mount_weight_unit
                t_weight = LSM_weight + mount_weight + compressor

            return t_weight

            def LSM(conductor_vol,beta,conductor_den,len_pod ):
                weight = conductor_vol*beta*conductor_den*len_pod
                return weight

            def LIM(perm_vol, alpha, perm_den,len_pod):
                weight = perm_den*alpha*perm_vol*len_pod
                return weight



        def suspension_weight(lev_mounts, gamma,psi,  halbach_units, len_pod):
            t_weight = lev_mounts*psi*len_pod + gamma*halbach_units*len_pod
            return t_weight



        def power_electronics(battery, motor, p_mad):
            t_weight = battery + motor + p_mad
            return t_weight

        def payload(passenger,baggage, interior, safety):
            t_weight = passenger + baggage + interior + safety
            return t_weight

        def exterior_shell(volume_pod, pod_den):
            t_weight = volume_pod*pod_den
            return t_weight





    def external_weight(nu, pylon_den,vol_pylon ):


        def pylon_weight(nu, pylon_den, vol_pylon):
            t_weight = nu * pylon_den * vol_pylon
            return t_weight

        def tube_weight(tube_thickness, route_len, tube_den ):
            t_weight = tube_thickness * route_len * tube_den
            return t_weight

        def sub_track(track_vol, route_len):
            t_weight = track_vol * route_len
            return t_weight






if __name__ == '__main__':
    # set up problem
    root = Group()
    p = Problem(root)
    p.root.add('comp', total_weight())
    p.setup()
    p.root.list_connections()
    p.run()

    # print following properties
    print 'StackWeight(kg) : %f' % p['comp.StackWeight']
    print 'StackVol(m^3) : %f' % p['comp.StackVol']