// ****************************************************************************
// * ASDL 
// * Title: Internal Combustion Engine Tool
// * Author: Ryan Donnan
// *
// ****************************************************************************
void RunDeck(real altitude, real dTs, real startRPM, real maxRPM) {
	setOption("printStates","OFF");
	Amb.dTs = dTs;
	Amb.alt = altitude;
	engine.N= startRPM;
		while (engine.N <= maxRPM){
			cout << "RPM = "<< engine.N << endl;
			run(); decksheet.update();
			engine.N += 100;
			++CASE;
		}
}

void RunTrade(real startrc, real maxrc) {
	setOption("printStates","OFF");
	engine.r_c = startrc;
		while (engine.r_c <= maxrc){
			cout << "r_c = "<< engine.r_c << endl;
			run(); decksheet.update();
			engine.r_c += 1.0;
			++CASE;
		}
}

void VdTrade(real startHP, real maxHP) {
	setOption("printStates","OFF");
		while (startHP <= maxHP){
			Des_HP.eq_rhs = toStr(startHP);
			cout << "Des_HP = "<< Des_HP.eq_rhs << endl;
			autoSolverSetup(); 
			solver.addIndependent ( "Bore" );
			solver.addDependent ( "Des_HP" );
			run(); decksheet.update();
			startHP += 10.0;
			++CASE;
		}
}


//============================================================
//  NPSS Independents
//============================================================
Independent Comp_Ratio {  
	varName = "engine.r_c"; }

Independent BL_Ratio {  
	varName = "engine.BL"; }

Independent Bore { 
	varName = "engine.B"; }

Independent Stroke {
	varName = "engine.L"; }

Independent DispVol {
	varName = "engine.V_d"; }

Independent desRPM {
	varName = "engine.N"; }
	
Independent engine_W {
	varName = "engine.q"; }
	
//============================================================
//  NPSS Dependents and Constraints
//============================================================
Dependent Max_W {
  eq_lhs = "engine.q";
  eq_rhs = "1.0";
}

Dependent Des_HP {
  eq_lhs = "engine.P_b";
  eq_rhs = "151.137"; 
}

Dependent OffDes_HP {
  eq_lhs = "engine.P_b";
  eq_rhs = "151.137"; 
  constraintNameList = {"Max_W"};
  constraintPriorities = {1};
  limitTypes = {"MAX"};  
}
  
Dependent Vpiston {
  eq_lhs = "engine.c";
  eq_rhs = "20"; }
  
  
//===========================================================
//  Run Off-Design Engine Sweeps
//===========================================================

void run_OffDes_Sweeps(real Max_Power) {
	cout << "==============  ICE Design Properties ===========" << endl << endl;
	cout << "engine Weight = " << engine.dryWeight << endl;
	cout << "engine BSFC = " << engine.BSFC << endl;
	cout << "engine gal/hr = " << engine.BSFC * engine.P_b /6.01 << endl;
	cout << "engine Pb = " << engine.P_b << endl;
	cout << "engine Vd = " << engine.V_d * 100**3 << endl; 
	cout << "engine EGT (F) = " << (engine.EGT-273)*1.8+32 << endl;
	cout << endl;
	cout << "============== Run Off-Design ===================" << endl << endl;
	
	//Vectors to run overflow
	real Hp_Vec[] = {0.45, 0.55, 0.65, 0.75, 0.85, 1.0};
	real RPM_Vec[] = {2000, 2200, 2400, 2600, 2700};
	
	//OutFileStream
	OutFileStream OffDes_Sweeps;
	OffDes_Sweeps.open("Engine_OffDes");
	OffDes_Sweeps << "HP   RPM   SFC   q   W" << endl;

	//for loop to run the cases
	int i, j;
	for(i = 0; i < RPM_Vec.entries(); i++) {
		for(j = 0; j < Hp_Vec.entries(); j++) {
			OffDes_HP.eq_rhs = toStr(Hp_Vec[j] * Max_Power);
			engine.N = RPM_Vec[i];
			run();
			OffDes_Sweeps << engine.P_b << " ";
			OffDes_Sweeps << engine.N << " ";
			OffDes_Sweeps << engine.BSFC * engine.P_b / 6.01 << " ";
			OffDes_Sweeps << engine.q << " ";
			OffDes_Sweeps << engine.Fl_I.W << endl;			
		}
	}
	
}
