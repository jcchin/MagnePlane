"""
Calculate Lift based on Magnet Parameters!
inspired by: Jed Storey - http://mitrocketscience.blogspot.com/search/label/
and: Paul R. Friend - http://cegt201.bradley.edu/projects/proj2004/maglevt1/reason.html
"""

import numpy as np
from os import remove
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.pyplot as plt
from math import pi, atan, sin, cos, e, log

from openmdao.core.group import Group, Component, IndepVarComp
from openmdao.api import SqliteRecorder
from openmdao.solvers.newton import Newton
from openmdao.solvers.scipy_gmres import ScipyGMRES

from openmdao.solvers.ln_gauss_seidel import LinearGaussSeidel
from openmdao.solvers.ln_direct import DirectSolver


class Lift(Component):
    def __init__(self):
        super(Lift, self).__init__()
        self.ln_solver = LinearGaussSeidel()

        # Pod Inputs
        self.add_param('Br',
                       val=0.64,
                       units='Tesla',
                       desc='residual magnetic flux')  #0.64
        self.add_param(
            'd', val=0.009525,
            units='m', desc='magnet thickness')  #0.009525 - 0.00635
        self.add_param('N', val=16.0, desc='number magnets')
        self.add_param('M', val=4.0, desc='number magnets per halbach')
        self.add_param('Is',
                       val=3.0,
                       desc='number vertically oriented magnets')

        self.add_param('edge',
                       val=0.1524,
                       units='m',
                       desc='edge length of cube magnets being used')
        self.add_param('mass', val=0.375, units='kg', desc='pod mass')

        # Track Inputs (laminated track)
        self.add_param('delta_c',
                       val=0.0005334,
                       units='m',
                       desc='single layer thickness')
        self.add_param('strip_c',
                       val=0.0105,
                       units='m',
                       desc='center strip spacing')
        self.add_param('Pc', val=0.21, units='m', desc='width of track')
        self.add_param('rc',
                       val=171.3,
                       units='Ohm-m',
                       desc='electric resistivity')
        self.add_param('Nt',
                       val=0.005,
                       units='m',
                       desc='width of conductive strip')
        self.add_param('Ns', val=1, desc='number of laminated sheets')

        # Inductive Loading
        self.add_param('al',
                       val=0.00079 * 8.,
                       units='m',
                       desc='conductor bundle height loaded')
        self.add_param('wf',
                       val=0.0,
                       units='m',
                       desc='total ferrite tile width')

        # Pod/Track Relation
        self.add_param('y1', val=0.01, units='m', desc='rolling clearance')
        self.add_param('veloc', val=10.0, units='m/s', desc='pod velocity')

        self.add_param(
            'dia_out',
            val=0.0406,
            units='m',
            desc='diameter of largest magnet ring. Cenetered on magnets')
        self.add_param('dia_in',
                       val=0.0406,
                       units='m',
                       desc='diameter of smallest magnet ring')
        self.add_param('fill_frac',
                       val=0.8,
                       units='m',
                       desc='fraction of area filled by magnet')
        self.add_param('nr', val=1.0, desc='number of magnet rings')
        self.add_param('rpm', val=10000.0, desc='motor rotations / minute')
        self.add_param('t_plate',
                       val=0.25 * 0.0254,
                       units='m',
                       desc='conductive plate thickness')
        self.add_param('k', val=1., desc='constant for thin sheet')
        self.add_param('rho0',
                       val=2.82E-08,
                       units='Ohm-m',
                       desc='resistivity of aluminum at 20C')
        self.add_param(
            'alpha',
            val=0.0039,
            desc='constant for linear dependence of rho on temperature of aluminum.')
        self.add_param('T',
                       val=26.,
                       units='degC',
                       desc='temperature of aluminum')
        self.add_param('Den',
                       val=2700,
                       units='kg/m^3',
                       desc='density of aluminum')
        self.add_param('mu',
                       val=1.26E-06,
                       units='H/m',
                       desc='magnetic permeability of aluminum')
        self.add_param('halbach',
                       val=1.8,
                       desc='field multiplier for using halbach')
        self.add_param('height',
                       val=0.0051,
                       units='m',
                       desc='rotor lift height')

        # outputs
        # pod outputs
        self.add_output('lambda',
                        val=0.00635 * 4.,
                        units='m',
                        desc='halbach wavelength')
        self.add_output('B0', val=0.9, units='T', desc='halbach peak strength')
        self.add_output('area_mag',
                        val=0.4,
                        units='m',
                        desc='actual magnetized area')
        self.add_output('pforce',
                        val=5.,
                        units='N',
                        desc='required levitation force')
        # system outputs
        self.add_output('l1d',
                        val=5.7E-08,
                        units='Henrys',
                        desc='distributed "1 turn" inductance')
        self.add_output('li',
                        val=0.,
                        units='Henrys',
                        desc='added inductance from loading')
        self.add_output('l1',
                        val=5.7E-08,
                        units='Henrys',
                        desc='one turn inductance')
        self.add_output('r1',
                        val=0.0007,
                        units='Ohm',
                        desc='one turn resistance')
        self.add_output('rl_pole', val=12250, units='rad/s', desc='R/L Pole')
        self.add_output('omegaOsc',
                        val=47.,
                        units='rad/s',
                        desc='Oscillation frequency')
        self.add_output('vOsc',
                        val=0.41,
                        units='m/s',
                        desc='Oscillation velocity')
        # breakpoint analysis
        self.add_output('vb',
                        val=23.,
                        units='m/s',
                        desc='levitation breakpoint')
        self.add_output('sb',
                        val=52.,
                        units='mi/h',
                        desc='levitation breakpoint speed mph')
        self.add_output('omegab',
                        val=2650.,
                        units='rad/s',
                        desc='frequency breakpoint')
        self.add_output('Fxb',
                        val=17.,
                        units='N',
                        desc='drag force at breakpoint')
        self.add_output('l2db', val=0.2, desc='lift to drag at breakpoint')
        # transition analysis
        self.add_output('vt', val=47, units='m/s', desc='transition velocity')
        self.add_output('st',
                        val=52.,
                        units='mi/h',
                        desc='levitation transition speed mph')
        self.add_output('omegat',
                        val=2650.,
                        units='rad/s',
                        desc='frequency transition')
        self.add_output('Fxyt',
                        val=17.,
                        units='N',
                        desc='drag force at transition')
        self.add_output('Lht', val=17., units='m', desc='levitation height')
        self.add_output('l2dt', val=0.2, desc='lift to drag at transition')
        # user specified output point
        self.add_output('omegau', val=2650., units='rad/s', desc='frequency')
        self.add_output('su',
                        val=52.,
                        units='mi/h',
                        desc='levitation speed mph')
        self.add_output('Fyu', val=17., units='N', desc='levitation force')
        self.add_output('Fxu', val=17., units='N', desc='drag force')
        self.add_output('Lhu', val=17., units='m', desc='levitation height')
        self.add_output('Fyuf',
                        val=17.,
                        units='N',
                        desc='fixed levitation force')
        self.add_output('Fxuf', val=17., units='N', desc='fixed drag force')
        self.add_output('l2du', val=0.2, desc='lift to drag')

        self.add_output('rho',
                        val=2.88599E-08,
                        units='Ohm-m',
                        desc='resistivity of aluminum at elevated temps')
        self.add_output('n',
                        val=4.0,
                        desc='number of halbach cycles (4 magnets per cycle)')

        self.add_output('vol_mag',
                        val=0.4,
                        units='m',
                        desc='volume of plate with eddy current')
        self.add_output('omega',
                        val=16.0,
                        units='rad/s',
                        desc='rotation speed')
        self.add_output('area_ring',
                        val=0.0,
                        units='lbm/s',
                        desc='area of ring that contains all magnets')
        self.add_output('B',
                        val=0.172,
                        units='T',
                        desc='magnetic field at plate (Tesla')
        self.add_output('f',
                        val=4.0,
                        units='Hz',
                        desc='magnet rotation frequency')
        self.add_output('P_norm',
                        val=11214.5582,
                        units='W/kg',
                        desc='watts per mass dissipated in aluminum')
        self.add_output('P',
                        val=155.7289266,
                        units='W',
                        desc='watts dissipated in aluminum')
        self.add_output('delta',
                        val=0.003311459,
                        units='m',
                        desc='skin depth equation for good conductors. \
                                                point at which current density has fallen to 0.37 of surface. \
                                                make sure thickness of plate is at least 2*delta')

    def solve_nonlinear(self, p, u, r):  # params, unknowns, residuals

        edge = p['edge']
        height = p['height']
        u['lambda'] = p['M'] * p['d']
        lambdaa = u['lambda']

        # u['n'] = p['N'] / p['M']
        # u['omega'] = p['rpm'] * 2.0 * pi / 60.0
        # u['area_ring'] = pi * ((((p['dia_out']+edge)/2.)**2) - (((p['dia_out']-edge)/2.)**2))
        # u['vol_mag'] = u['area_ring'] * p['t_plate']
        # u['f'] = u['n'] * u['omega'] / (2 * pi)
        # #                                 *(ATAN((B6*B6)  /(2*   B20* SQRT(4*B20^2+B6^2+B6^2)))             -ATAN((B6*B6) /(2*(B6+B20)*     SQRT(4*(B6+B20)^2+B6^2+B6^2))))
        # u['B'] = (p['Br']/pi)*(atan((edge**2)/(2*height*(4*height**2+edge**2+edge**2)**0.5))-atan((edge*edge)/(2*(edge+height)*(4*(edge+height)**2+edge**2+edge**2)**0.5)))*p['halbach']
        # u['rho'] = p['rho0'] * (1+p['alpha']*(p['T']-20))
        # u['delta'] = (u['rho']/(pi * p['mu'] * u['f']))**0.5
        # u['P_norm'] = ((pi * u['B'] * p['t_plate'] * u['f'])**2) / (6. * p['k'] * u['rho'] * p['Den'])
        # u['P'] = u['P_norm'] * u['vol_mag'] * p['Den']

        u['B0'] = p['Br'] * (1. - e**(-2. * pi * p['d'] / lambdaa)) * (
            (sin(pi / p['M'])) / (pi / p['M']))
        u['pforce'] = p['mass'] * 9.81
        u['area_mag'] = 0.0036  #u['area_ring'] * p['fill_frac']
        u['l1d'] = 4 * pi * 10** -7 * p['Pc'] / (4 * pi * p['strip_c'] /
                                                 lambdaa)

        trackArea = p['delta_c'] * p['Nt']
        R = p['rc'] * p['Pc'] / (u['area_mag'] * p['Ns']) * 10** -10
        levc = p['delta_c'] * p['Ns'] / 2.
        p['y1'] = 0.01 + levc

        Q = (e**(pi * p['al'] / lambdaa) + e**
             (-pi * p['al'] / lambdaa)) / (e**(pi * p['al'] / lambdaa) - e**
                                           (-pi * p['al'] / lambdaa))
        u['li'] = (p['wf'] / p['Pc']) * (Q - 1.) * u['l1d']
        #     levs = (p['edge']/p['Pc'])*(p['l1d']/(Ll+p['l1d']));
        #     levsl = (p['l1d']/(Ll+p['l1d']));
        levsl = 1.

        u['l1'] = u['l1d'] + u['li']

        # Fringe Effects
        if p['Is'] == 0:
            levsf = 1
        else:
            levsf = 1 - 1 / (2 * p['Is'])
        # Scale Factor
        levs = levsf * levsl

        u['rl_pole'] = u['r1'] / u['l1']

        # Oscillation Frequency and Velocity
        u['omegaOsc'] = (4 * pi * 9.81 / lambdaa)**0.5
        u['vOsc'] = u['omegaOsc'] * lambdaa / (2 * pi)

        # Break Point Calculations:
        u['omegab'] = u['r1'] / (u['l1'] * (((
            (levs * u['B0']**2. * p['edge'] /
             (4. * pi * u['l1'] * p['strip_c'] / lambdaa)) * u['area_mag'] * e
            **(-4. * pi * p['y1'] / lambdaa)) / u['pforce']) - 1)**0.5)
        u['vb'] = (u['omegab'] * lambdaa) / (2. * pi)
        u['sb'] = u['vb'] * (100 / 2.54) * 1 / (5280 * 12) * 60 * 60
        u['Fxb'] = levs * (u['B0']**2. * p['edge'] /
                           (4 * pi * u['l1'] * p['strip_c'] / lambdaa)) * (
                               (u['r1'] / (u['omegab'] * u['l1'])) /
                               (1 + (u['r1'] / (u['omegab'] * u['l1']))**2.)
                           ) * e**(-4 * pi * p['y1'] / lambdaa) * u['area_mag']
        u['l2db'] = u['omegab'] * u['l1'] / u['r1']

        # Transition Calculations (Lift = Drag):
        u['omegat'] = u['r1'] / u['l1']
        u['vt'] = u['omegat'] * lambdaa / (2. * pi)
        u['st'] = u['vt'] * (100. / 2.54) * 1. / (5280. * 12.) * 60. * 60.
        u['Lht'] = log(
            (u['pforce'] /
             (levs * (u['B0']**2. * p['edge'] /
                      (4. * pi * u['l1'] * p['strip_c'] / lambdaa)) *
              (1 / (1 + (u['r1'] / (u['omegat'] * u['l1']))**2.)) *
              u['area_mag']))) * (lambdaa / (-4. * pi)) - levc
        #    Fxyt = levs*(Bo^2*w/(4*pi*L*dc/lambdaa))*((R/(u['omegat']*L))/(1+(R/(u['omegat']*L))^2))*exp(-4*pi*(Lht+levc)/lambdaa)*A;
        u['Fxyt'] = levs * (u['B0']**2. * p['edge'] /
                            (4. * pi * u['l1'] * p['strip_c'] / lambdaa)) * (
                                (u['r1'] / (u['omegat'] * u['l1'])) /
                                (1. + (u['r1'] /
                                       (u['omegat'] * u['l1']))**2.)) * e**(
                                           -4. * pi *
                                           (p['y1']) / lambdaa) * u['area_mag']
        u['l2dt'] = u['omegat'] * u['l1'] / u['r1']

        # User Input:
        if p['veloc'] == 0:
            u['omegau'] = 0
            u['su'] = 0
            Fyu = 0
            Fxu = 0
            Lhu = 0
            L2Du = 0
        else:
            u['omegau'] = 2 * pi * p['veloc'] / lambdaa
            u['su'] = p['veloc'] * (100. / 2.54) * 1. / (5280. *
                                                         12.) * 60. * 60.
            u['Lhu'] = log(
                (u['pforce'] /
                 (levs * (u['B0']**2. * p['edge'] /
                          (4. * pi * u['l1'] * p['strip_c'] / lambdaa)) *
                  (1 / (1 + (u['r1'] / (u['omegau'] * u['l1']))**2.)) *
                  u['area_mag']))) * (lambdaa / (-4. * pi)) - levc
            u['Fyu'] = levs * (u['B0']**2. * p['edge'] / (
                4. * pi * u['l1'] * p['strip_c'] / lambdaa)) * (1. / (
                    1. + (u['r1'] / (u['omegau'] * u['l1']))**2)) * e**(
                        -4. * pi * (u['Lhu'] + levc) / lambdaa) * u['area_mag']
            u['Fxu'] = levs * (u['B0']**2. * p['edge'] / (
                4. * pi * u['l1'] * p['strip_c'] / lambdaa)) * (
                    (u['r1'] / (u['omegau'] * u['l1'])) /
                    (1. + (u['r1'] / (u['omegau'] * u['l1']))**2.)) * e**(
                        -4 * pi * (u['Lhu'] + levc) / lambdaa) * u['area_mag']
            u['Fyuf'] = levs * (u['B0']**2. * p['edge'] / (
                4. * pi * u['l1'] * p['strip_c'] / lambdaa)) * (1. / (
                    1. + (u['r1'] / (u['omegau'] * u['l1']))**2)) * e**(
                        -4. * pi * (p['y1']) / lambdaa) * u['area_mag']
            u['Fxuf'] = levs * (u['B0']**2. * p['edge'] / (
                4. * pi * u['l1'] * p['strip_c'] / lambdaa)) * (
                    (u['r1'] / (u['omegau'] * u['l1'])) /
                    (1. + (u['r1'] / (u['omegau'] * u['l1']))**2.)) * e**(
                        -4. * pi * (p['y1']) / lambdaa) * u['area_mag']

            u['l2du'] = u['omegau'] * u['l1'] / u['r1']

        vp = []
        Fyp1 = []
        Fxp1 = []
        Fxp2 = []
        Fyp2 = []
        Fyp3 = []
        Fxp3 = []
        Fyp4 = []
        Fxp4 = []
        mFxp2 = []
        mFyp2 = []
        mFxp4 = []
        mFyp4 = []
        mag = []
        mag4 = []
        phase = []
        phase4 = []
        Lhp = []
        w = lm = p['edge']
        tm = p['mass']
        dc = p['strip_c']

        vtp = round(u['vt'] / 2.) * 30.
        vstep = vtp / 100.
        vp = np.arange(vstep, vtp + vstep, vstep)

        A = lm * w
        tf = tm * 9.81

        R = u['r1']
        L = u['l1']
        y1 = p['y1']
        Bo = u['B0']
        for n in xrange(100):
            omegap = 2 * pi * vp[n - 1] / lambdaa
            Lh_p = log(
                (tf / (levs * (Bo**2. * w / (4. * pi * L * dc / lambdaa)) *
                       (1. / (1. +
                              (R /
                               (omegap * L))**2.)) * A))) * (lambdaa /
                                                             (-4. * pi)) - levc
            Lhp.extend([Lh_p])
            if ((y1 - levc) >= Lh_p):
                Fyp1.extend([levs * (Bo**2. * w /
                                     (4. * pi * L * dc / lambdaa)) *
                             (1. / (1. + (R / (omegap * L))**2.)) * e**
                             (-4. * pi * (y1) / lambdaa) * A])
                Fyp3.extend([levs * (Bo**2. * w /
                                     (4. * pi * L * dc / lambdaa)) *
                             (atan(omegap * L / R) / (pi / 2.)) * e**
                             (-4. * pi * (y1) / lambdaa) * A])
                Fxp1.extend([levs * (Bo**2. * w /
                                     (4. * pi * L * dc / lambdaa)) * (
                                         (R / (omegap * L)) /
                                         (1. + (R / (omegap * L))**2.)) * e**
                             (-4. * pi * (y1) / lambdaa) * A])
                Fxp3.extend([levs * (Bo**2. * w /
                                     (4. * pi * L * dc / lambdaa)) *
                             (1. - atan(omegap * L / R) / (pi / 2.)) * e**
                             (-4. * pi * (y1) / lambdaa) * A])
            else:
                Fyp1.extend([levs * (Bo**2. * w /
                                     (4. * pi * L * dc / lambdaa)) *
                             (1. / (1. + (R / (omegap * L))**2.)) * e**
                             (-4. * pi * (Lh_p + levc) / lambdaa) * A])
                Fyp3.extend([levs * (Bo**2. * w /
                                     (4. * pi * L * dc / lambdaa)) *
                             (atan(omegap * L / R) / (pi / 2.)) * e**
                             (-4. * pi * (Lh_p + levc) / lambdaa) * A])
                Fxp1.extend([levs * (Bo**2. * w /
                                     (4. * pi * L * dc / lambdaa)) * (
                                         (R / (omegap * L)) /
                                         (1. + (R / (omegap * L))**2.)) * e**
                             (-4. * pi * (Lh_p + levc) / lambdaa) * A])
                Fxp3.extend([levs * (Bo**2. * w /
                                     (4. * pi * L * dc / lambdaa)) *
                             (1. - atan(omegap * L / R) / (pi / 2.)) * e**
                             (-4. * pi * (Lh_p + levc) / lambdaa) * A])

            Fyp2.extend([levs * (Bo**2. * w / (4. * pi * L * dc / lambdaa)) *
                         (1. / (1. + (R / (omegap * L))**2.)) * e**
                         (-4. * pi * (y1) / lambdaa) * A])
            Fxp2.extend([levs * (Bo**2. * w / (4. * pi * L * dc / lambdaa)) * (
                (R / (omegap * L)) / (1 + (R / (omegap * L))**2.)) * e**
                         (-4. * pi * (y1) / lambdaa) * A])
            mFyp2.extend([20. * log(Fyp2[n - 1])])
            mFxp2.extend([20. * log(Fxp2[n - 1])])
            mag.extend([20. * log(Fxp2[n - 1] + Fyp2[n - 1])])
            phase.extend([-90. * Fyp2[n - 1] / (Fyp2[n - 1] + Fxp2[n - 1])])

            Fyp4.extend([levs * (Bo**2. * w / (4. * pi * L * dc / lambdaa)) *
                         (atan(omegap * L / R) / (pi / 2.)) * e**
                         (-4. * pi * (y1) / lambdaa) * A])
            Fxp4.extend([levs * (Bo**2. * w / (4. * pi * L * dc / lambdaa)) *
                         (1. - atan(omegap * L / R) / (pi / 2.)) * e**
                         (-4. * pi * (y1) / lambdaa) * A])
            mFyp4.extend([20. * log(Fyp4[n - 1])])
            mFxp4.extend([20. * log(Fxp4[n - 1])])
            mag4.extend([20. * log(Fxp4[n - 1] + Fyp4[n - 1])])
            phase4.extend([-90. * Fyp4[n - 1] / (Fyp4[n - 1] + Fxp4[n - 1])])

        M1 = []
        d1 = np.zeros(50)
        percentLambda = np.zeros(50)
        volume = np.zeros(50)
        mass = np.zeros(50)
        B01 = np.zeros((100, 50))
        B0mass = np.zeros((100, 50))
        Lh = np.zeros((100, 50))
        Fy = np.zeros((100, 50))
        FyMass = np.zeros((100, 50))
        Br = p['Br']

        for q in xrange(1, 101, 1):
            M1.extend([q])
            for n in xrange(1, 51, 1):
                d1[n - 1] = (lambdaa * n * 0.01)
                percentLambda[n - 1] = (0.01 * n)
                volume[n - 1] = (d1[n - 1]**3.)
                mass[n - 1] = (7453.7002 * volume[n - 1])
                B01[q - 1, n - 1] = Br * (1. - e**
                                          (-2. * pi * d1[n - 1] / lambdaa)) * (
                                              (sin(pi / M1[q - 1])) /
                                              (pi / M1[q - 1]))
                B0mass[q - 1, n - 1] = B01[q - 1, n - 1] / mass[n - 1]
                Lh[q - 1, n - 1] = log(((tf + mass[n - 1] * M1[q - 1] * 9.81) /
                                        (levs * (B01[q - 1, n - 1]**2. * w /
                                                 (4. * pi * L * dc / lambdaa))
                                         * A))) * (lambdaa / (-4. * pi)) - levc
                Fy[q - 1, n - 1] = levs * (B01[q - 1, n - 1]**2. * w / (
                    4. * pi * L * dc / lambdaa)) * e**(-4. * pi * (
                        Lh[q - 1, n - 1]) / lambdaa) * A
                FyMass[q - 1, n - 1] = Fy[q - 1, n - 1] / mass[n - 1]

        lamda2 = []
        d2 = []
        volume2 = []
        mass2 = []
        B02 = []
        Lh2 = []
        for z in xrange(1, 500):
            lamda2.extend([z * 0.005])
            d2.extend([lamda2[z - 1] / 5.])
            volume2.extend([d2[z - 1]**3])
            mass2.extend([7453.7002 * volume2[z - 1]])
            B02.extend([Br * (1. - e**(-2. * pi * d2[z - 1] / lambdaa)) * ((
                sin(pi / 5.)) / (pi / 5.))])
            Lh2.extend([log((
                (tf + mass2[z - 1] * 5. * 9.81) /
                (levs * (B02[z - 1]**2. * w /
                         (4. * pi * L * dc / lamda2[z - 1])) * A))) *
                        (lamda2[z - 1] / (-4. * pi))])

        Bx = np.zeros((120, 20))
        By = np.zeros((120, 20))
        B0 = u['B0']
        x = 0
        y = 0
        for n in xrange(1, 120):
            for o in xrange(1, 20):
                Bx[n, o] = B0 * sin(
                    (2. * pi / lambdaa) * x) * e**(-(2. * pi / lambdaa) * y)
                By[n, o] = B0 * cos(
                    (2. * pi / lambdaa) * x) * e**(-(2. * pi / lambdaa) * y)
                y = y + 0.001
            y = 0
            x = x + 0.001

        x = np.arange(0, .12, 0.001)
        y = np.arange(0, .02, 0.001)
        X, Y = np.meshgrid(x, y)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot_surface(Y.T, X.T, Bx, cmap=cm.coolwarm, shade=True)
        plt.title('Bx')
        ay = fig.add_subplot(212, projection='3d')
        ay.plot_surface(Y.T, X.T, By, cmap=cm.coolwarm, shade=True)
        plt.title('By')
        plt.show()

        fig2, (ax2, ay2) = plt.subplots(2, sharex=True)
        #ax2 = fig2.add_subplot(111)

        ax2.plot(vp[:], Fxp1, vp[:], Fyp1, vp[:], Fxp2[:], vp[:],
                 Fyp2[1:] + Fyp2[-1:])
        ax2.set_ylabel('Newtons')
        ax2.set_xlabel('Velocity (meters/sec)')
        ax2.set_title(
            'Drag (red) Docked (g) & Levitation (teal) Docked (b) Forces')
        ax2.set_ylim([0, 100])
        #ay2 = fig2.add_subplot(212)
        result = Lhp[1:] + Lhp[-1:]
        ay2.plot(vp, [x * 39.3701 for x in result])
        plt.ylabel('Height (inches)')
        plt.show()

        #         If the magnet radius is 2" (0.05m) then for 100 m/s

        # v/r = omega
        # 100 / 0.05 = 2000 rad/s = 19,000rpm

        # Our large motor only goes up to 3,552rpm

        M1m, Lhm = np.meshgrid(M1, np.arange(1, 51))

        fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # ax.plot_surface(percentLambda, M1m.T, Lh, cmap=cm.coolwarm, shade=True)
        # plt.title('Optimum Magnet Thickness')
        # plt.ylabel('Number of Magnets M')
        # plt.xlabel('Magnet Thickness d as %/ lamda')
        ay = fig.add_subplot(212)
        ay.plot(lamda2, Lh2)
        plt.title('Maximum Levitation Height for Wavelength')
        plt.xlabel('Wavelength lamda')
        plt.ylabel('Levitation Height y')
        plt.show()


if __name__ == "__main__":
    from openmdao.core.problem import Problem

    root = Group()
    root.add('lift', Lift())

    p = Problem(root)

    recorder = SqliteRecorder('maglev')
    recorder.options['record_params'] = True
    recorder.options['record_metadata'] = True
    p.driver.add_recorder(recorder)

    p.setup()
    p.run()
    p.root.dump()

    import sqlitedict
    from pprint import pprint

    db = sqlitedict.SqliteDict('maglev', 'openmdao')
    print(db.keys())
    data = db['rank0:Driver/1']
    u = data['Unknowns']
    pprint(u)
    remove('./maglev')
    #print('n: ', p['lift.n'])

    # print('n: ', p['lift.n'])
    # print('omega: ', p['lift.omega'])
    # print('area_ring: ', p['lift.area_ring'])
    # print('area_mag: ', p['lift.area_mag'])
    # print('vol_mag: ', p['lift.vol_mag'])
    # print('f: ', p['lift.f'])
    # print('B-: ', p['lift.B'])
    # print('rho: ', p['lift.rho'])
    # print('delta: ', p['lift.delta'])
    # print('P_norm: ', p['lift.P_norm'])
    # print('P: ', p['lift.P'])

    if 2 * p['lift.delta'] > p['lift.t_plate']:
        print('plate is thick enough')
    else:
        print('plate should be thicker (see delta)')
