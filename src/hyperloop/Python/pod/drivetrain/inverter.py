import numpy as np
from openmdao.api import Component


class Inverter(Component):
    """The `Inverter` class represents a BLDC inverter in an OpenMDAO model

    The `Inverter` class models the efficiency loss across a typical BLDC
    inverter following the example from [1]_.

    Params
    ------
    inverter_efficiency : float
        power out / power in (W)
    output_voltage : float
        amplitude of AC output voltage (A)
    output_current : float
        amplitude of AC output current (A)
    output_frequency : float
        frequency of AC output (Hz)
    input_voltage : float
        amplitude of DC input voltage (V)

    Outputs
    -------
    input_current : float
        amplitude of DC input current (A)
    input_power : float
        amplitude of DC input current (W)

    References
    ----------
    .. [1] Gladin, Ali, Collins, "Conceptual Modeling of Electric and Hybrid-Electric Propulsion for UAS Applications"
       Georgia Tech, 2015
    """

    def __init__(self):
        """Initializes a `Inverter` object

        Sets up the given Params/Outputs of the OpenMDAO `Inverter` component, initializes their shape, and
        sets them to their default values.
        """
        super(Inverter, self).__init__()

        self.add_param('inverter_efficiency', 1.0, desc='power out / power in')
        self.add_param('output_voltage',
                       120.0,
                       desc='amplitude of AC output voltage',
                       units='V')
        self.add_param('output_current',
                       2.0,
                       desc='amplitude of AC output current',
                       units='A')
        self.add_param('output_frequency',
                       60.0,
                       desc='frequency of AC output',
                       units='Hz')
        self.add_param('input_voltage',
                       100.,
                       desc='amplitude of DC input voltage',
                       units='V')

        self.add_output('input_current',
                        0.48,
                        desc='amplitude of DC input current',
                        units='A')
        self.add_output('input_power', 10.0, units='W')

    def solve_nonlinear(self, params, unknowns, resids):
        """Runs the `Battery` component and sets its respective outputs to their calculated results

        Args
        ----------
        params : `VecWrapper`
            `VecWrapper` containing parameters

        unknowns : `VecWrapper`
            `VecWrapper` containing outputs and states

        resids : `VecWrapper`
            `VecWrapper` containing residuals

        output_power = params['output_voltage'] * params[
            'output_current'] * 3.0 * np.sqrt(2.0 / 3.0)
        """
        output_power = params['output_voltage'] * params[
            'output_current'] * 3.0 * np.sqrt(2.0 / 3.0)

        # TODO perform efficiency lookup
        unknowns['input_power'] = output_power / params['inverter_efficiency']
        unknowns['input_current'] = unknowns['input_power'] / params[
            'input_voltage']
