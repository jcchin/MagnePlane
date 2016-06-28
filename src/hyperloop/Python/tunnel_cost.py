"""
Current tunnel cost estimation very rough. Needs refinement.
Default parameters taken from Hyperloop Alpha.
"""

from __future__ import print_function
from openmdao.core.problem import Problem
from openmdao.core.group import Group
from openmdao.core.component import Component
import math
from collections import namedtuple
from .tools import io_helper

class DefaultsHandler(object):
    def __init__(self):
        pass

    # default format for namedtuple factory method
    global param
    param = namedtuple('Param', 'name val desc unit')

    diam = param('diameter', 2.23, 'diameter of tunnel', 'm')
    len = param('length', 563.00, 'length of tunnel', 'km')

    # note that 'cost' is NOT in the config file, an example of a var that is defined but is not in config file
    # note that 'cost' starts at 0 (and conceivably always SHOULD start with 0) this is something that could
    # possibly be defined in an add_param call below, and avoid being redefined here.
    cost = param('cost', 0.0, 'total cost of tunnel', 'USD')

    # this could be factored out into a helper class/static method
    def get_config_from_file(self, config):
        for attr in vars(DefaultsHandler):
            attr_var = getattr(self, attr)
            # magic: parses out functions and inherited Python fields
            if not attr.startswith('__') and not callable(attr_var):
                if attr_var.name in config:
                    # use str() to parse out the pesky 'u' prefix in Unicode strings in Python 2.x
                    new_attr = param(attr_var.name,
                                     float(config[attr_var.name]['val']),
                                     str(config[attr_var.name]['desc']),
                                     str(config[attr_var.name]['unit']))
                    setattr(self, attr, new_attr)
                    print(getattr(self, attr))


class TunnelCost(Component):
    """
    Params
    ------
    diameter : float
        Diameter of tunnel in meters. Default value is 2.23.
    length : float
        Length of tunnel in km. Default value is 563.00, distance from SF to LA.

    Returns
    -------
    cost : float
        Total cost of tunnel in USD. Default value is 0.0.

    Notes
    ----------
    [1] Rostami, Jamal, Mahmoud Sepehrmanesh, Ehsan Alavi Gharahbagh, 
    and Navid Mojtabai. "Planning Level Tunnel Cost Estimation Based on 
    Statistical Analysis of Historical Data." Tunnelling and Underground 
    Space Technology 33 (2013): 22-33. Web. 
    <https://www.researchgate.net/publication/233926915_Planning_level_tunnel_cost_estimation_based_on_statistical_analysis_of_historical_data>.
    """
    global defaults

    def __init__(self, config=None):
        super(TunnelCost, self).__init__()

        global defaults
        defaults = DefaultsHandler()

        if config is not None:
            defaults.get_config_from_file(config)


        # default inner diameter for passenger tube from Hyperloop Alpha
        self.add_param(defaults.diam.name, defaults.diam.val, desc=defaults.diam.desc, units=defaults.diam.unit)
        # default tunnel length from SF to LA
        self.add_param(defaults.len.name, defaults.len.val, desc=defaults.len.desc, units=defaults.len.unit)

        self.add_output(defaults.cost.name, defaults.cost.val, desc=defaults.cost.desc, units=defaults.cost.unit)

    # formula taken from conventional subway excavation data
    def solve_nonlinear(self, params, unknowns, resids):

        # TODO for now the below regression model depends on having correct units, I believe?, this should be changed for future.
        assert defaults.len.unit == 'km'
        assert defaults.diam.unit == 'm'
        assert defaults.cost.unit == 'USD'

        # TODO for final publish store all citations in common document not inline
        # formula taken from conventional subway excavation data
        # https://www.researchgate.net/publication/233926915_Planning_level_tunnel_cost_estimation_based_on_statistical_analysis_of_historical_data
        unknowns[defaults.cost.name] = 1000000 * math.pow(10, (1.10 + (0.933 * math.log10(params[defaults.len.name])) + (0.614 * math.log10(params[defaults.diam.name]))))


    def print_results(self):
        print("{} ({}): {}".format(defaults.diam.name, defaults.diam.unit, defaults.diam.val))
        print("{} ({}): {}".format(defaults.len.name, defaults.len.unit, defaults.len.val))
        print("{} ({}): {}".format(defaults.cost.name, defaults.cost.unit, defaults.cost.val))

if __name__ == '__main__':
    p = Problem(root=Group())

    # play with these two lines:
    # x = TunnelCost(config=io_helper.InputHelper('default.JSON').get_config('tunnel_data'))
    x = TunnelCost()

    p.root.add('comp', x)
    p.setup()
    p.root.list_connections()
    p.run()
    x.print_results()
