//=============================================
//
//    Mission.fnc
//	  Created by:  J. Gladin
//	  Last Edited by: Manish Pokhrel
//	  Date: 09-02-2015
//
//=============================================

void  Run_Mission() {		
		
	//Input Mission Segment Numbers	
	cout << endl << "------ Running Mission ------" << endl << endl;

	//Set Hover Condition
	Amb.alt = 3000.; 	//ft
	Amb.dTs = 0.0;   	//deg. F
	R22.V_kts = 0.0;  	//knots
	real Time_Seg = 5.0;//minutes   
	real DegH_Seg = DOH_Hover;  

	
	//Start Time Step
	real timeTotal = 0;
	real totalFuel = 0;
	real Cruise_Range = 0;
	real deltaFuel;
	real Extended_Cruise_Range = 0;
	
	//Set power levels for ICE and Battery		
	if(DegH_Seg == 0.0) {
		setup_all_gas();
	} else if(DegH_Seg == 1.0) {
		setup_all_electric();
	} else {
		setup_split();
	}
	
	real saveStateOfCharge = Battery1.StateOfCharge;
	
	cout << "Running First Hover Segment...";
	
	//==== Running Hover ====//
	while((Battery1.StateOfCharge > Battery1.CapDisLimit*100. || DegH_Seg == 0.0) && timeTotal  < Time_Seg) {				
		//Set Power
		R22.calculate();		
		setup_power(DegH_Seg, R22.Ptot);		
		run();	
		
		//Update Time, StateOfCharge, Fuel Flow, and Weight
		timeTotal += Battery1.dischargeInterval;		
		Battery1.StateOfCharge = Battery1.NewStateOfCharge;
		if(DegH_Seg != 1.0) {
			deltaFuel = engine.BSFC * Battery1.dischargeInterval / 60. * engine.P_b; //lbs
			R22.W -= deltaFuel;
			totalFuel += deltaFuel;
		}
		
	}
	
	cout << "Finished!" << endl;
	
	//Reset the state of charge if all gas just so it doesn't cause the loops to stop
	if(DegH_Seg == 0.0) {Battery1.StateOfCharge = saveStateOfCharge;}
	
	//End if out of charge
	if(Battery1.StateOfCharge <= Battery1.CapDisLimit*100 && DegH_Seg != 0.0) {	
		cout << "Finished!  Ended During First Hover!" << endl << endl;
		cout << "==== Performance ====" << endl;
		cout << "Battery Weight " << Battery1.StackWeight * 2.2 << endl; //lbs.
		cout << "PP Weight " << Motor1.Mass * 2.2 + engine.dryWeight << endl;
		cout << "Fuel burned " << totalFuel << endl; // lbs.
		cout << "Range " << Cruise_Range << endl; // miles		
		quit(); //end the run	
	}
	
	real hoverEndStateOfCharge = 100.; //Initial Guess
	int missionComplete = 0; //Initial Value
	real Cruise_Time_Seg; //minutes -- Initial Guess
	real Lower_Bound = 0;  
	real Upper_Bound = Initial_Cruise_Time * 2.0;

	//Save the state of the vehicle after first hover
	real saveFuel = totalFuel;
	real saveWeight = R22.W;
	
	//Iteration tracker
	int iter = 1;
	
	saveStateOfCharge = Battery1.StateOfCharge;

	while(!missionComplete) {
		
		//Reset the battery and fuel burned to after the first hover
		Battery1.StateOfCharge = saveStateOfCharge;		
		totalFuel = saveFuel;
		R22.W = saveWeight;
		
		//Update the cruise segment time based on the updated bounds
		Cruise_Time_Seg = 0.5 * (Lower_Bound + Upper_Bound);
		
		//==== Running Cruise ====//
		
		//Set Cruise Condition
		Amb.alt = 3000.; //ft
		Amb.dTs = 0.0;    //deg. F
		R22.V_kts = Cruise_Speed;

		DegH_Seg = DOH_Cruise; 	
		
		Time_Seg = Cruise_Time_Seg;
		
		//Reset time counter
		timeTotal = 0;
		Cruise_Range = 0;
		
		R22.calculate(); 
		saveStateOfCharge = Battery1.StateOfCharge;
		real saveCruiseSFC, saveCruisePower;
		if(Time_Seg > .001) {
			//Run Cruise Segment
			//cout << "Running Cruise Segment...";
			while((Battery1.StateOfCharge > Battery1.CapDisLimit*100. || DegH_Seg == 0.0) && timeTotal  < Time_Seg) {				
				//Set Power
				R22.calculate();
				setup_power(DegH_Seg, R22.Ptot);
				// setup_power(DegH_Seg, 83);
				// cout << R22.Ptot << endl;
				run(); 
				saveCruiseSFC = engine.BSFC;
				saveCruisePower = engine.P_b;
				//cout << engine.BSFC << endl;
				//Update time, cruise range, battery SoC, Fuel burned, and Vehicle Weight
				timeTotal += Battery1.dischargeInterval;	
				Battery1.StateOfCharge = Battery1.NewStateOfCharge;
				Cruise_Range += Battery1.dischargeInterval / 60. * R22.V_kts;
				if(DegH_Seg != 1.0) {
					deltaFuel = engine.BSFC * Battery1.dischargeInterval / 60. * engine.P_b; //lbs
					R22.W -= deltaFuel;
					totalFuel += deltaFuel;
				}
			}
			
			int Range_Extend = 0;
			Extended_Cruise_Range = 0;
			
			//Range Extension
			if(Range_Extend) {				
				while(timeTotal  < Initial_Cruise_Time) {
					R22.calculate();
					setup_power(0, R22.Ptot);					
					run();
					timeTotal += Battery1.dischargeInterval;	
					Extended_Cruise_Range += Battery1.dischargeInterval / 60. * R22.V_kts;
					deltaFuel = engine.BSFC * Battery1.dischargeInterval / 60. * engine.P_b; //lbs
					R22.W -= deltaFuel;
					totalFuel += deltaFuel;				
				}				
			}
			
			cout << "Finished!" << endl<<endl;
		} else {
			Time_Seg = 0;
		}		
		
		if(DegH_Seg == 0.0) {Battery1.StateOfCharge = saveStateOfCharge;}
		
		//Set Hover Condition
		Amb.alt = 3000.; //ft
		Amb.dTs = 0.0;    //deg. F
		R22.V_kts = 0.0;  //knots
		Time_Seg = 5.0; //minutes
		DegH_Seg = DOH_Hover; 

		//Start Time Step
		timeTotal = 0;
		
		//==== Running Hover ====//
		cout << "Running Final Hover Segment..."<<endl;
		while((Battery1.StateOfCharge > Battery1.CapDisLimit*100. || DegH_Seg == 0.0) && timeTotal  < Time_Seg) {				
			//Set Power
			R22.calculate();
			setup_power(DegH_Seg, R22.Ptot);

			run();
			Battery1.StateOfCharge = Battery1.NewStateOfCharge;
			timeTotal += Battery1.dischargeInterval;				
		}
		//End if out of charge
		cout << "Finished!" << endl;

		hoverEndStateOfCharge = Battery1.StateOfCharge;
		
		iter++;
		cout << "Battyer1.StateOfCharge = " << Battery1.StateOfCharge << endl;
		cout << "timeTotal = " << timeTotal << endl;
		//Update the Bounds:  If all conditions are met or cruise range is zero then mission is done
		if(((Battery1.StateOfCharge <= Battery1.CapDisLimit*100 || DOH_Cruise == 0) && timeTotal >= Time_Seg) || Cruise_Time_Seg <= .001) {
			missionComplete = 1;			
		} else if(timeTotal >= Time_Seg){  //If the time is satisfied but battery isn't discharged, then 
			Lower_Bound = Cruise_Time_Seg; //Increase the cruise time
			cout << "Excess capacity -- Increasing cruise time:  New Time = " << 0.5 * (Upper_Bound + Lower_Bound) << endl;
		} else if(Battery1.StateOfCharge <= Battery1.CapDisLimit*100 ){ //If the time isn't satisfied then 
			Upper_Bound = Cruise_Time_Seg; //Decrease the cruise time
			cout << "Capacity Limited -- Decreasing cruise time:  New Time = " << 0.5 * (Upper_Bound + Lower_Bound) << endl;
		}				
	}
	
	cout << endl << "----- Mission Complete! -----" << endl << endl;
	cout << "==== Performance ====" << endl;
	cout << "Battery Weight " << Battery1.StackWeight<< endl; //lbs.
	cout << "Motor Weight " << Motor1.Mass << endl;
	cout << "Power Electronics Weight " <<Max_Motor_Power/(1000*PtoWratio_PE)<<endl;
	cout << "Engine Weight " << engine.dryWeight/2.2 << endl;
	cout << "Fuel burned " << totalFuel/2.2 << endl; // kg
	cout << "Reserve Fuel weight " << saveCruiseSFC * 0.5 * saveCruisePower/2.2 <<endl; //kg
	cout<<"Fuel Tank Weight "<< 0.05*(totalFuel/2.2)<<endl;  // lbs
	cout << "Hybrid Range " << (Cruise_Range)*1.850 << endl; // miles	
	cout << "Extended Range " << (Extended_Cruise_Range)*1.850 << endl; // miles	
	cout << "Total Range " << (Cruise_Range+Extended_Cruise_Range)*1.850 << endl; // miles	
	cout<<endl;

	real totalweight;
	if(DOH_Cruise == 0 && DOH_Hover == 0) {
		totalweight = (totalFuel + 0.05*(totalFuel+saveCruiseSFC * 0.5 * saveCruisePower) + engine.dryWeight + saveCruiseSFC * 0.5 * saveCruisePower)/2.2 ;			
	} else if(DOH_Cruise == 1 && DOH_Hover == 1)  {
		totalweight = (Battery1.StackWeight + Motor1.Mass + Max_Motor_Power/(1000*PtoWratio_PE) );
	} else {
		totalweight = (Battery1.StackWeight * 2.2 + Motor1.Mass * 2.2 + Max_Motor_Power/(1000*PtoWratio_PE)*2.2 + totalFuel + 0.05*(totalFuel+saveCruiseSFC * 0.5 * saveCruisePower)+ engine.dryWeight+saveCruiseSFC * 0.5 * saveCruisePower)/2.2 ;		
	}
	cout<< "Total weiht of this system is (in kg) "<< totalweight<<endl;
	cout<<"Total Payload Capacity (in kg) "<<(778/2.2-totalweight)<<endl;
}

void setup_all_electric() {
	solver.clear();
	solver.addIndependent("I0");
	solver.addIndependent("Inverter1.ind_Voltage");
	solver.addIndependent("Battery1.ind_Current");
	
	solver.addDependent("eta_Dep");
	solver.addDependent("Bus.Dependent1");
	solver.addDependent("DCTrans.dep_Power");
	
	solver.addIndependent("R0");
	solver.addDependent("R_Dep");

}

void setup_all_gas() {
	solver.clear();
	
	solver.addDependent("Des_HP");
	solver.addIndependent("engine_W");
	
}

void setup_split() {
	solver.clear();
	
	solver.addDependent("Des_HP");
	solver.addIndependent("engine_W");
	
	solver.addIndependent("I0");
	solver.addIndependent("Inverter1.ind_Voltage");
	solver.addIndependent("Battery1.ind_Current");	
	solver.addDependent("eta_Dep");
	solver.addDependent("Bus.Dependent1");
	solver.addDependent("DCTrans.dep_Power");
	
	solver.addIndependent("R0");
	solver.addDependent("R_Dep");

}


void setup_power(real DegH, real Ptot) {
	
	if(DegH == 1) {
		Motor1.Torque = Ptot / Shaft1.Nmech * C_HP_PER_RPMtoFT_LBF;
	} else if(DegH == 0.0) {
		Des_HP.eq_rhs = toStr(Ptot);
	} else {
		Motor1.Torque = Ptot * ( 30 / PI)  / Shaft1.Nmech * 550. * DegH;
		Des_HP.eq_rhs = toStr((1.-DegH)*Ptot);
	}
	
}
