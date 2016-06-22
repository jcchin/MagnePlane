// ****************************************************************************
// * ASDL 
// * Title: Inds and Deps for the electric Motor Setup
// * Author: Jonathan Gladin
// *
// ****************************************************************************

Independent I0 {varName = "Motor1.I0";}
Independent I0_Des {varName = "Motor1.I0_Des";}
Independent R0 {varName = "Motor1.R0";}
Independent As {varName = "Motor1.As";}
Independent Motor_Torque {varName = "Motor1.Torque";}

Dependent R_Dep {
	eq_rhs = "Motor1.R0";
	eq_lhs = "Motor1.Rcalc";
}

Dependent I0_Dep {
	eq_rhs = "Motor1.I0";
	eq_lhs = "Motor1.I0_Des";
}

Dependent eta_Dep {
	eq_rhs = "Motor1.Current*Motor1.Voltage";
	eq_lhs = "Motor1.P_input";
}

Dependent eta_Calibrate {
	eq_rhs = "Motor1.efficiency";
	eq_lhs = "0.84";
}

Dependent Motor_Power {
	eq_rhs = "R22.Ptot * DegH_Seg";
	eq_lhs = "Motor1.Torque * Shaft1.Nmech * (PI / 30.) / 550.";
}

void gen_motor_map(real minRPM, real  maxRPM, real  minQ, real  maxQ) {

	//Generate Motor Map
	real numNpnts  =  100;
	real numQpnts  =  100;
	OutFileStream DataOut ;
	DataOut.filename = "Data.Out";
	
	OutFileStream InverterDataOut;
	InverterDataOut.open("InverterData.Out");	
	int i, j;

	for(i = 0; i<numNpnts; i++) {
		for(j = 0; j<numQpnts; j++) {			
			Shaft1.Nmech = minRPM + toReal(i)/(numNpnts-1.)*(maxRPM-minRPM);
			Motor1.Torque = minQ+ toReal(j)/(numQpnts-1.)*(maxQ-minQ); 

			if(Motor1.Torque > 0 ) {Motor1.switchGenerator = "MOTOR";} else {Motor1.switchGenerator = "GENERATOR";}
			
			run();		
			
			if( abs(Motor1.P_mech/746.) < Motor1.DesignPower ) {
				DataOut << Shaft1.Nmech << " " << Motor1.Torque << " " << Motor1.efficiency << endl;
				InverterDataOut << Shaft1.Nmech << " " << Motor1.Torque << " " << Inverter1.Efficiency << endl;
				cout << Motor1.Torque << " " << Inverter1.S_Efficiency.percent_load << " " << Inverter1.S_Efficiency.modulation << " " << Inverter1.Efficiency << endl;
			}			
		}	
	}
	system("C:\Python34\python.exe plot_map.py");
}
