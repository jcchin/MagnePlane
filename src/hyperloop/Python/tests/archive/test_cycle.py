import numpy as np
import unittest
import os
import sys
from openmdao.api import Problem, Group, IndepVarComp
from pycycle.engines.hyperloop import hyperloop

fpath = os.path.dirname(os.path.realpath(__file__))
#ref_data = np.loadtxt(fpath + '/data/hyperloop.csv',
#                      delimiter=',', skiprows=1)
# headers to extract from csv file
#header = ['inlet.eRamBase','comp.PR','comp.eff','burn.FAR','burn.eff','turb.eff','turb.PR','nozzle.Cfg','shaft.Nmech','amb.MN','amb.alt','start.W','start.Pt','start.Tt','start.ht','start.s','start.MN','start.V','start.A','inlet.W','inlet.Pt','inlet.Tt','inlet.ht','inlet.s','inlet.MN','inlet.Fram','inlet.V','inlet.A','comp.W','comp.Pt','comp.Tt','comp.ht','comp.s','comp.MN','comp.V','comp.A','burn.W','burn.Pt','burn.Tt','burn.ht','burn.s','burn.MN','burn.V','burn.A','turb.W','turb.Pt','turb.Tt','turb.ht','turb.s','turb.MN','turb.V','turb.A','nozzle.W','nozzle.Pt','nozzle.Tt','nozzle.ht','nozzle.s','nozzle.MN','nozzle.PsExh','nozzle.Fg','nozzle.V','nozzle.A','shaft.trqIn','shaft.trqOut','shaft.trqNet','shaft.pwrIn','shaft.pwrOut','shaft.pwrNet']
#h_map = dict(((v_name,i) for i,v_name in enumerate(header)))

tol = 3.0e-2
np.set_printoptions(linewidth=500)

# element names, npss vs pycycle
#npss_comps = ["inlet", "comp", "burn", "turb", "nozzle"]
pyc_comps = ["inlet", "comp", "duct", "nozz"]

# flow variables to compare for each element name above
#npss_flow_vars = ["Pt","Tt","ht","s","W", "MN","V","A"]
pyc_flow_vars = ["Fl_O:tot:P", "Fl_O:tot:T", "Fl_O:tot:h", "Fl_O:tot:S",
                 "Fl_O:stat:W", "Fl_O:stat:MN", "Fl_O:stat:V",
                 "Fl_O:stat:area"]

# non-flow variables to check for each case
#npss_names_nonflow = ['shaft.pwrIn', 'shaft.pwrOut', 'nozzle.Fg', 'inlet.Fram']
pyc_names_nonflow = ['hyperloop.shaft.pwr_in', 'hyperloop.shaft.pwr_out',
                     'hyperloop.nozz.Fg', 'hyperloop.inlet.F_ram']


def rel_err(a, b):
    if abs(a) > 0:
        err = abs(a - b) / a
    else:
        err = abs(b)
    return err


def make_model():
    """
    Returns set-up instance of on-design hyperloop model
    """
    prob = Problem()
    prob.root = Group()
    prob.root.add('hyperloop', CompressionCycle())  # JP-7 or 'Jet-A(g)'

    params = (('P', 17., {'units': 'psi'}), ('T', 500.0, {'units': 'degR'}),
              ('W', 1.0, {'units': 'lbm/s'}),
              ('alt', 35000.0, {'units': 'ft'}), ('MN', 0.8, {'units': ''}),
              ('Ps_exhaust', 14.7, {'units': 'lbf/inch**2'}))
    prob.root.add('des_vars', IndepVarComp(params))
    prob.root.connect('des_vars.W', 'hyperloop.fc.fs.W')
    prob.root.connect('des_vars.MN', 'hyperloop.fc.MN_target')
    prob.root.connect('des_vars.Ps_exhaust', 'hyperloop.nozz.Ps_exhaust')

    prob.setup(check=False)

    return prob


def test_generator():
    """hyperloop unit tests"""

    def check_values(err, path1, path2):
        """comparator method - yielding this generates a test case dynamically"""

        print("NPSS %s vs Pycycle %s\n" % (path1, path2))
        print("Relative error: %f, tolerance: %f" % (err, tol))
        print(
            "Preceding flow stations: (* indicates failed check against NPSS)\n")
        print(flow_str)
        assert abs(err) < tol

    for casenum, data in enumerate(ref_data):
        # get fresh model instance
        prob = make_model()

        # set parameter values from the csv
        prob['des_vars.W'] = 1.0
        prob['hyperloop.fc.alt'] = 30001.0
        prob['des_vars.MN'] = 0.6
        prob['hyperloop.inlet.ram_recovery'] = 0.99
        prob['hyperloop.comp.map.PRdes'] = 4.9
        prob['hyperloop.comp.map.effDes'] = 0.92
        #prob['hyperloop.burner.Fl_I:FAR'] = data[h_map['burn.FAR']]
        #prob['hyperloop.turb.eff_design'] = data[h_map['turb.eff']]
        prob['hyperloop.shaft.Nmech'] = 15000
        prob['hyperloop.nozz.Cfg'] = 0.99
        prob['des_vars.Ps_exhaust'] = 4.2

        # for name1, name2 in zip(npss_comps[:-1], pyc_comps[:-1]):
        #     path2 =  "hyperloop.%s.%s" % (name2, "MN_target")
        #     path1 = name1 + ".MN"
        #     prob[path2] = data[h_map[path1]]

        # run the model instance to compute outputs
        prob.run()

        # flow variables at all exit flowstations
        # base_row = "{:<15}" + len(npss_flow_vars) * " {:<10.6}" + "\n"
        # vals = ["Element"] + npss_flow_vars
        # flow_str = ""
        # flow_str += base_row.format(*vals)

        n_vars = len(npss_flow_vars)
        flow_str += 100 * "=" + "\n"
        for i in range(len(npss_comps)):
            v1 = []
            v2 = []
            flow_str_old = flow_str
            for k in range(n_vars):
                path1 = "%s.%s" % (npss_comps[i], npss_flow_vars[k])
                npss = data[h_map[path1]]
                path2 = "hyperloop.%s.%s" % (pyc_comps[i], pyc_flow_vars[k])
                pyc = prob[path2]

                err = rel_err(npss, pyc)
                if err > tol:
                    pyc = "* " + str(pyc)
                    npss = "  " + str(npss)

                v1.append(npss)
                v2.append(pyc)

                vals1 = [npss_comps[i] + "(NPSS)"] + v1 + (n_vars - k) * ["-"]
                vals2 = [npss_comps[i] + "(Pyc.)"] + v2 + (n_vars - k) * ["-"]
                flow_str = flow_str_old + base_row.format(
                    *vals1) + base_row.format(*vals2)
                yield check_values, err, path1, path2

            vals1 = [npss_comps[i] + "(NPSS)"] + v1
            vals2 = [npss_comps[i] + "(Pyc.)"] + v2
            flow_str = flow_str_old + base_row.format(
                *vals1) + base_row.format(*vals2)
            flow_str += 100 * "-" + "\n"

        # some non flow variable checks too

        flow_str += "\n"
        flow_str += "Non-flow variable check:\n"
        flow_str += "| {:<15} {:<15} {:<15} |\n".format("Variable", "NPSS",
                                                        "Pyc.")
        flow_str += 15 * 4 * "-" + "\n"
        flow_str_old = flow_str
        for path1, path2 in zip(npss_names_nonflow, pyc_names_nonflow):
            npss = data[h_map[path1]]
            pyc = prob[path2]
            flow_str = flow_str_old + "| {:<15} {:<15} {:<15} |\n".format(
                path1, npss, pyc)
            err = rel_err(npss, pyc)
            yield check_values, err, path1, path2

        break


if __name__ == "__main__":
    import nose
    nose.runmodule()
