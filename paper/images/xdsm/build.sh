#!/bin/bash

python flowpath_xdsm.py
python cycle_xdsm.py
python chem_eq_xdsm.py
python motor_xdsm.py
python drivetrain_xdsm.py
python temp_xdsm.py
python levitation_xdsm.py
python pod_xdsm.py
python steady_state_vacuum_xdsm.py
python tube_and_pod_xdsm.py
python tube_xdsm.py


rm *.aux *.log *.DS_Store *.pyc
