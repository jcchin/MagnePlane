''' Pointwise wrapper '''
import os

# --- Python/system level imports
from subprocess import call
from string import Template

# --- OpenMDAO main and library imports
from openmdao.api import Problem, Group, ExternalCode, IndepVarComp


class Pointwise(ExternalCode):
    ''' OpenMDAO component for executing Pointwise '''

    # -------------------------------------------
    # --- File Wrapper/Template for Pointwise ---
    # -------------------------------------------
    def __init__(self, *args, **kwargs):

        # -------------------------------------------------
        # --- Constructor for Surface Meshing Component ---
        # -------------------------------------------------
        super(Pointwise, self).__init__()

        self.options['external_input_files'] = [
            os.path.join('..', 'Meshing', 'AFLR3', 'Hyperloop_PW.b8.ugrid'),
            os.path.join('..', 'Meshing', 'AFLR3', 'Hyperloop.tags')
        ]
        self.options['external_output_files'] = [os.path.join(
            '..', 'Aero', 'Fun3D', 'Hyperloop.b8.ugrid')]
        self.options['command'] = [
            os.path.join('/Applications', 'Pointwise', 'PointwiseV17.3R4',
                         'macosx', 'Pointwise.app', 'Contents', 'MacOS',
                         'tclsh8.5'),
            os.path.join('..', 'Meshing', 'Pointwise', 'Hyperloop-Mesher.glf')
        ]

        # --------------------------------------
        # --- External Code Public Variables ---
        # --------------------------------------
        self.force_execute = True

    def execute(self):

        call([
            '/Applications/Pointwise/PointwiseV17.3R4/macosx/Pointwise.app/Contents/MacOS/tclsh8.5',
            self._filein
        ])

        # -----------------------------------
        # --- Execute Pointwise Component ---
        # -----------------------------------
        super(Pointwise, self).solve_nonlinear(params, unknowns, resids)


if __name__ == "__main__":

    # -------------------------
    # --- Default Test Case ---
    # -------------------------
    p = Problem(root=Group())
    p.root.add('pointwise', Pointwise())
    p.setup()
    p.run()
