#   OpenMDAO main and library imports
from openmdao.core.api import Assembly

#   OpenMDAO component imports
#from Freestream import Freestream
#from Fun3D import Fun3D
#from openCSM import OpenCSM

from cycle_wrapper import CycleWrap
from pointwise import Pointwise
from aflr3 import AFLR3

class Sim(Assembly):

    def configure(self):

        # --------------------------------------------------------------------------- #
        #   Freestream Component
        # --------------------------------------------------------------------------- #
        self.add('freestream', Freestream())

        # --------------------------------------------------------------------------- #
        #    Inlet Design Component
        # --------------------------------------------------------------------------- #
        #self.add('supin', SUPIN())

        # --------------------------------------------------------------------------- #
        #    Thermo Cycle Component
        # --------------------------------------------------------------------------- #
        self.add('cycle', cycle_wrapper()) # NPSS
        #self.add('cycle', cycle()) # Pycycle

        # --------------------------------------------------------------------------- #
        #    Geometry Component
        # --------------------------------------------------------------------------- #
        #self.add('esp', ESP()) # Engineering Sketch Pad (uses .csm files)
        #self.add('solidworks', SolidWorks()) # Solidworks

        # --------------------------------------------------------------------------- #
        #    Pointwise Mesher Component
        # --------------------------------------------------------------------------- #
        self.add('pointwise', Pointwise())

        # --------------------------------------------------------------------------- #
        #    AFLR3 Mesher Component
        # --------------------------------------------------------------------------- #
        self.add('aflr3', AFLR3())

        # --------------------------------------------------------------------------- #
        #    Fun3D CFD Component
        # --------------------------------------------------------------------------- #
        self.add('fun3d', Fun3D())

        # --------------------------------------------------------------------------- #
        #    Liner Induction Motor Component
        # --------------------------------------------------------------------------- #
        # --------------------------------------------------------------------------- #
        #    Cost Component
        # --------------------------------------------------------------------------- #
        self.add('tube_cost', TubeCost()) # Tom Gregory Model

        # --------------------------------------------------------------------------- #
        #    Mission Workflow
        # --------------------------------------------------------------------------- #
        # Rob Faulk

        # --------------------------------------------------------------------------- #
        #    Power Component
        # --------------------------------------------------------------------------- #
        self.add('battery', Battery())
        # add more here for vacuum, lim

        # --------------------------------------------------------------------------- #
        #    Tube Structure Component
        # --------------------------------------------------------------------------- #
        self.add('struct', TubeStructural())

        # --------------------------------------------------------------------------- #
        #    Vacuum Component
        # --------------------------------------------------------------------------- #
        self.add('vacuum', Vacuum())

        # --------------------------------------------------------------------------- #
        #    Tube Thermal Component
        # --------------------------------------------------------------------------- #
        self.add('thermal', TubeTemp())



        # --------------------------------------------------------------------------- #
        #   Create Main Assembly Workflow
        # --------------------------------------------------------------------------- #
        #   Add component instances to top-level assembly
        # self.driver.workflow.add(['freestream', 'npss', 'supin', 'opencsm', 'pointwise', 'aflr3', 'cart3d', 'fun3d'])

        #self.driver.workflow.add(['freestream', 'supin', 'opencsm', 'pointwise', 'aflr3', 'fun3d'])
        self.driver.workflow.add(['freestream', 'supin', 'fun3d'])

        # --------------------------------------------------------------------------- #
        #   Create Data Connections
        # --------------------------------------------------------------------------- #
        # self.connect('freestream.alt', ['supin.Freestream.Alt', 'cart3d.alt', 'fun3d.alt'])
        # self.connect('freestream.M_inf', ['supin.Freestream.Mach', 'cart3d.M_inf', 'fun3d.M_inf'])
        # self.connect('freestream.p_inf', ['supin.Freestream.Pres', 'cart3d.p_inf', 'fun3d.p_inf'])
        # self.connect('freestream.t_inf', ['supin.Freestream.Temp', 'cart3d.t_inf', 'fun3d.t_inf'])
        # self.connect('freestream.alpha', ['supin.Freestream.Alpha', 'cart3d.alpha', 'fun3d.alpha'])
        # self.connect('freestream.d_inf', ['cart3d.d_inf', 'fun3d.d_inf'])

        # self.connect('supin.Outputs.pt2_ptL', ['cart3d.pt2_ptL', 'fun3d.pt2_ptL'])
        # self.connect('supin.Outputs.tt2_ttL', ['cart3d.tt2_ttL', 'fun3d.tt2_ttL'])
        # self.connect('supin.Outputs.M2', ['cart3d.M2', 'fun3d.M2'])
        # self.connect('supin.Outputs.A2', ['cart3d.A2', 'fun3d.A2'])
        # self.connect('supin.Outputs.mdot', ['cart3d.mdot', 'fun3d.mdot'])
        # self.connect('supin.Outputs.yEF', ['opencsm.yEF'])
        # self.connect('supin.Outputs.Rspin', ['opencsm.Rspin'])

        # if Inlet == "STEX":
        #     if Grid == "Euler":
        #         self.opencsm._filein = '../ESP/LBFD-STEX.template'
        #         copy_files('../ESP/Inviscid/*', '../ESP/')
        #         copy_files('../ESP/Inviscid/STEX/*', '../ESP/')
        #     else:
        #         self.opencsm._filein = '../ESP/LBFD-STEX.template'
        #         copy_files('../ESP/Viscous/*', '../ESP/')
        #         #copy_files('../ESP/Viscous/STEX-Inlet/*', '../ESP/')
        #         copy_files('../ESP/Viscous/STEX/*', '../ESP/')

        #     self.pointwise._filein = '../Pointwise/Load-STEX.glf'

        # elif Inlet == "AxiSpike":
        #     if Grid == "Euler":
        #         self.opencsm._filein = '../ESP/LBFD-AxiSpike.template'
        #         copy_files('../ESP/Inviscid/*', '../ESP/')
        #         copy_files('../ESP/Inviscid/AxiSpike/*', '../ESP/')
        #     else:
        #         self.opencsm._filein = '../ESP/LBFD-AxiSpike.template'
        #         copy_files('../ESP/Viscous/*', '../ESP/')
        #         #copy_files('../ESP/Viscous/AxiSpike-Inlet/*', '../ESP/')
        #         copy_files('../ESP/Viscous/AxiSpike/*', '../ESP/')
        #     self.pointwise._filein = '../Pointwise/Load-AxiSpike.glf'

if __name__ == '__main__':
