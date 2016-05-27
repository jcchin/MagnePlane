# MagnePlane (Hyperloop_v2)

"It's more of a plane without wings, than a train without wheels"

Expanding upon previous work

https://github.com/jcchin/hyperloop/tree/scitech

https://github.com/narwhaltribe/Hyperloop

Apache V2 License

========

Dev forks can be found here:

[Chris Heath](https://github.com/cmheath/MagnePlane)

[Jeff Chin](https://github.com/jcchin/MagnePlane)

[Ken Moore](https://github.com/Kenneth-T-Moore/MagnePlane)

[Rob Faulk](https://github.com/robfalck/MagnePlane)

========
### File Structure

* AFLR3 - 
  * Hyperloop.aflr3.arg -
  * Hyperloop.tags - 
  * Hyperloop_PW.mapbc -
* ESP -
  * capsule.csm
  * capsule.udc
  * flow_domain.csm
  * tube.udc
* Fun3D - 
  * Hyperloop.mapbc
  * fun3d.nml
  * fun3d.template
  * qscript
* Pointwise - 
  * Hyperloop-Mesher.glf
  * Hyperloop.mapbc
  * Hyperloop.pw
* Geometry -
  * csm
    * hyperloop.csm
    * hyperloop2.csm
    * inlet_test.csm
  * Hyperloop.SLDPRT
* Python - 
  * LIM.py - Linear Induction Motor Calcs [Not Started]
  * aero.py - Fun 3d Wrapper
  * aflr3.py - AFRL External Code Wrapper
  * aflr3.script - Shell Meshing Script
  * air_bearing.py - Basic Air Bearing Calcs [Deprecate]
  * api.py - Defined python imports
  * battery.py - Battery Calculations
  * cycle.py - Re-write of NPSS cycle in Pycycle
  * cycle_wrapper.py - Python File wrap around NPSS
  * inlet.py - Calculates the dimensions for the inlet and compressor entrance
  * levitation.py - Halbach Array Lift/Drag Calculations
  * mass.py - Vehicle Mass estimations
  * mission.py - Extremely Basic Mission Calcs [to be replaced]
  * pax_cabin.py - passenger cabin sizing
  * pod.py - Vehicle Assembly
  * tube_cost.py - Port of Tom Gregory's initial cost calculations
  * tube_limit_flow.py - Isentropic flow relations for determining flow speed around pod
  * tube_structure.py - Tube structural calcs [Placeholder]
  * tube_wall_temp.py - Calculates steady state temperature within the tube
  * vacuum.py - Vacuum Pump calculations
  * tests - test scripts
    * test_cycle.py - unit tests for pycycle `cycle.py` model
    * test_temp.py - unit tests for `tube_wall_temp.py`
* NPSS - Numerical Propulsion System Simulation (Thermodynamic Cycle Analysis Tool)
  * area.view_page - "Viewer" file defining output
  * magneplane.mdl - Actual cycle model
