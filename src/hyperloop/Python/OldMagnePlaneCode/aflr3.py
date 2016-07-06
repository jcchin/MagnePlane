''' AFLR3 wrapper '''
import os

# --- OpenMDAO imports
from openmdao.api import Problem, Group, ExternalCode, IndepVarComp


class AFLR3(ExternalCode):
    ''' OpenMDAO component for executing AFLR3 '''

    aflr3_exec = 'aflr3.script'

    # ---------------------------------------
    # --- File Wrapper/Template for AFLR3 ---
    # ---------------------------------------

    def __init__(self, *args, **kwargs):

        # -------------------------------------------------
        # --- Constructor for Surface Meshing Component ---
        # -------------------------------------------------
        super(AFLR3, self).__init__()

        self.options['external_input_files'] = [
            os.path.join('..', 'Meshing', 'AFLR3', 'Hyperloop_PW.b8.ugrid'),
            os.path.join('..', 'Meshing', 'AFLR3', 'Hyperloop.tags')
        ]
        self.options['external_output_files'] = [os.path.join(
            '..', 'Aero', 'Fun3D', 'Hyperloop.b8.ugrid')]
        self.options['command'] = ['sh', self.aflr3_exec]

    def execute(self):

        # -------------------------------
        # --- Execute AFLR3 Component ---
        # -------------------------------
        super(AFLR3, self).solve_nonlinear(params, unknowns, resids)


if __name__ == "__main__":

    # -------------------------
    # --- Default Test Case ---
    # -------------------------
    p = Problem(root=Group())
    p.root.add('aflr3', AFLR3())
    p.setup()
    p.run()
