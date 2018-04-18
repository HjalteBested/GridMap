/**	\file RobotMPC.cpp
 * \brief Linear MPC controller for Path Following Mobile Robot 
 * in the Presence of Velocity Constraints.
 *
 * \author Hjalte Bested Møller
 * \date 6. Marts 2018	
*/

#include "RobotMPC.h"

/// Construct object with default parameters.
RobotPathFollowMPC::RobotPathFollowMPC(){
	sys = LTIsys();
	mpc = LMPC(&sys);
	initRobotDone = false;
	
	// Default Values (DTU's SMR)
	float w = 0.26;
	float a = 0.10;
	initRobot(w,a);
	
	// Default Constraints
	float vw_max =  1;
	float vw_min = -vw_max;
	float omega_max = 2*PI/12;
	float omega_min = -omega_max;
	float v_max =  1;
	float v_min = -1;
	float acc_max =  0.3;
	float acc_min = -0.1;
	setConstraints(vw_min,vw_max,omega_min,omega_max,v_min,v_max,acc_min,acc_max);

	// More control parameters
	v_des = 0.3;
	a_des = 0.2;
	ks = v_des/omega_max;
    ka = 0;

    T = 0.04;
	sysType = 1;
	clkDiv = 1;
	useNLScaling = 1;

	// Init
    init(T*clkDiv, v_des);
	cout << "Initialization Done!" << endl;
}

/// Destroy object - currently a no operation
RobotPathFollowMPC::~RobotPathFollowMPC(){
	// cout << "RobotPathFollowMPC Object Was Destroyed!!" << endl;
}

/// Initialize robot, LTI system, etc.
///	T: 		Sampling Interval [s]
///	v_des: 	Desired Velocity [m/s] - can be changes at runtime
///	w: 		Wheel-to-wheel distance [m]
///	a: 		Reference point offset from wheel-axle along $\f x^r \f$ [m]
void RobotPathFollowMPC::init(float arg_T, float arg_v_des){
	// Initflags
	allDone = false;
	readyToCompute = false;
	firstRun = true;

	// LTI System
	if(sys.nx() == 0 || arg_v_des != v_des || arg_T != T){
		T = arg_T;
		v_des = arg_v_des;
		initSys(sysType, T, a, v_des, 0, 0);
	}

	// Init State Variables
	currentLine = 0;
	pose << 0,0,0;
	yk << -1e6, 0, 0, 0, 0, 0;
	uw << 0,0;
	ur << 0,0;

  	kStep = -1;
  	th_old = 0;
	th_nwrap = 0;
	timeSinceRefChange = 0;
	clkTick = 0;
}

/// Initialize Robot
void RobotPathFollowMPC::initRobot(float arg_w, float arg_a){
	w = arg_w;	// wheel-wheel distance
	a = arg_a;	// Reference point offset from wheel-axle along x^r
	Fw << 0.5, 0.5, -1/w, 1/w;
 	invFw = Fw.inverse();
 	// Init Constraints
	useConstrains = 1;
 	P << 	invFw, 
		   -invFw, 
			0,  1,
    		0, -1,
    		1,  0,
   		   -1,  0;
   	initRobotDone = true;
}

/// Initialize LTI System
void RobotPathFollowMPC::initSys(int type, float T, float a, float v0, float omega0, float th_err0){
	float cs = cos(th_err0);
	float sn = sin(th_err0);
	switch(type){
		case 1:{
			Matrix2f A;
			Vector2f Bu;
			Vector2f BuBar;
			Matrix2f Br;
			Matrix2f C;
			A  <<  	1, T*(v0*cs-a*omega0*sn),
			 		0, 1;
			Bu  << 	(T*(2*a+T*v0)*cs-T*a*omega0*sn)/2, 
		 			T;
			Br <<  	0, -T*(v0*cs-a*omega0*sn), 
					0, 0;
			C  <<   1, 0, 0, 1;
			BuBar = Bu*v0;
			sys.initBuBrC(A,BuBar,Br,C);
		}
		break;
		/*
		case 2:{ // LTI system OK - But not working with controller, yet!
			float tau_v = 0.5;
			float tau_omega = 0.5;
			float e_v     = exp(-T/tau_v);
			float e_omega = exp(-T/tau_omega);
			nx = 4; // Number of states
			nu = 2; // Number of controlled inputs
		    nr = 2; // Number of uncontrolled referenc inputs
		    nz = 2;	// Number of system outputs
		  	A.resize(nx,nx);
		    B.resize(nx,nu);
		    Bbar.resize(nx,nu);
		    Br.resize(nx,nr);
		    C.resize(nz,nx);   
		    A  <<  	1, T*(v0*cs-a*omega0*sn), sn*tau_v*(1-e_v), -((e_omega-1)*(a*sn*omega0-cs*v0)*tau_omega+((e_omega-1)*a-T*v0)*cs+T*a*sn*omega0)*tau_omega,
			 		0, 1, 0, -tau_omega*(e_omega-1),
			 		0, 0, e_v, 0,
			 		0, 0, 0, e_omega;
			B  << 	(e_omega-1)*(a*sn*omega0-cs*v0)*tau_omega*tau_omega+0.5*(((2*e_omega-2)*a-2*T*v0)*cs+2*T*a*sn*omega0)*tau_omega-0.5*T*((-T*v0-2*a)*cs+T*a*sn*omega0), sn*((e_v-1)*tau_v+T),
					tau_omega*(e_omega-1)+T, 0,
					0, 1-e_v,
					1-e_omega, 0;
			Br <<  	0, T*(a*omega0*sn-cs*v0), 
					0, 0,
					0, 0,
					0, 0;
			C  <<   1, 0, 0, 0, 
					0, 1, 0, 0;
			Bbar = B;
			Bbar.col(0) *= v0;
		}
		break;
		*/
	}
}

void RobotPathFollowMPC::initMPC(int N){
		mpc.initMPC(N);
		// Stacked Vectors for the prediction horizon
		Vk 		= VectorXf::Zero(mpc.N);
		Omegak 	= VectorXf::Zero(mpc.N);
		Dk 		= VectorXf::Zero(mpc.N+1);
		Sk 		= VectorXf::Zero(mpc.N+1);
		Thetak 	= VectorXf::Zero(mpc.N+1);
}

/// Design MPC
void RobotPathFollowMPC::design(float argQth, float argQdu, float argQu){
	Vector2f Qz; 	Qz 	<< 1, argQth;
	Vector1f Qdu; 	Qdu << argQdu;
	Vector1f Qu; 	Qu 	<< argQu;
	mpc.condenseStateSpace();
	mpc.designMPC(Qz,Qdu,Qu);
}

// Functions for accessing the needed reference info
Vector2f RobotPathFollowMPC::getPi(int i){
	int j = currentLine+i;
	int lastRow = numLines-1;
	if(j > lastRow) j = lastRow;
	return lineDefs.block(j,0,1,2).transpose();
}

Vector2f RobotPathFollowMPC::getPij(int i){
	int j = currentLine+i;
	int lastRow = numLines-1;
	if(j > lastRow) j = lastRow;
	return lineDefs.block(j,3,1,2).transpose();
}

float RobotPathFollowMPC::getLineLength(int i){
	int j = currentLine+i;
	int lastRow = numLines-1;
	if(j > lastRow) j = lastRow;
	return lineDefs(j,2);
}

float RobotPathFollowMPC::getPhi(int i){
	int j = currentLine+i;
	int lastRow = numLines-1;
	if(j > lastRow) j = lastRow;
	return lineDefs(j,5);
}

float RobotPathFollowMPC::getThetaErr(int i){
	int j = currentLine+i;
	int lastRow = numLines-1;
	if(j > lastRow) j = lastRow;
	// The angle of the line in radians
	float phi = lineDefs(j,5);
	// The angle of the robot
	float theta = pose(2);
	// The error angle of the robot with respect to the line
	return wrapToPi(theta-phi);
}

float RobotPathFollowMPC::getPhiFixed(int i){
	int j = currentLine+i;
	int lastRow = numLines-1;
	if(j > lastRow) j = lastRow;
	// The angle of the line in radians
	float phi = lineDefs(j,5);
	// The angle of the robot
	float theta = pose(2);
	// The error angle of the robot with respect to the line
	float theta_err = wrapToPi(theta-phi);
	// Ensure that varphi_ij is such that theta_err is in (-pi,pi] while maintaining theta linear.
	return theta-theta_err;
}

float RobotPathFollowMPC::getPsi(int i){
	int j = currentLine+i;
	int lastRow = numLines-1;
	if(j > lastRow) j = lastRow;
	return lineDefs(j,6);
}


VectorXf RobotPathFollowMPC::getPhiVec(){
	int N = mpc.N;
	VectorXf phiVec(N);
	for(int i=0; i<N; i++){
		phiVec(i) = mpc.Rk(2*i+1);
	}
	return phiVec;
}

// Precompute LineDef : (Pi(0),Pi(1), ||Pj-Pi||, p_ij(0), p_ij(1), varPhi_ij)
void RobotPathFollowMPC::setWaypoints(MatrixX2f waypoints){
	readyToCompute = false;
	int rows = waypoints.rows();
	lineDefs.resize(rows,Eigen::NoChange);
	lineDefs.leftCols(2) = waypoints;
	makeLineDefs();
}

void RobotPathFollowMPC::clearWaypoints(){
	readyToCompute = false;
	lineDefs.resize(0,Eigen::NoChange);
	lineDefs.setZero();
	numLines = 0;
	currentLine = 0;
	allDone = false;
}

void RobotPathFollowMPC::addWaypoint(float x, float y){
	int row = lineDefs.rows();
	lineDefs.conservativeResize(row+1,Eigen::NoChange);
	lineDefs(row,0) = x;
	lineDefs(row,1) = y;
	makeLineDefs();
}

void RobotPathFollowMPC::insertWaypoint(float i, float x, float y){
	int rows = lineDefs.rows();
	MatrixX7f top 	 =  lineDefs.topRows(i);
	MatrixX7f bottom =  lineDefs.bottomRows(rows-i);
	lineDefs.resize(rows+1,Eigen::NoChange);
	// Insert Top / Bottom in new matrix
	lineDefs.topRows(i) = top;
	lineDefs.bottomRows(rows-i) = bottom;
	// Insert new point
	lineDefs(i,0) = x;
	lineDefs(i,1) = y;
	// Recompute line definitions
	makeLineDefs();
}

void RobotPathFollowMPC::insertWaypointRel(float i, float x, float y){
	// insertWaypoint(currentLine+1,pose(0),pose(1));
	insertWaypoint(currentLine+i,x,y);
}

// Precompute LineDef : (Pi(0),Pi(1), ||Pj-Pi||, p_ij(0), p_ij(1), varPhi_ij, psi_j)
void RobotPathFollowMPC::makeLineDefs(){
	int M = lineDefs.rows();
	if(M>1){
		Vector2f Pi;
		Vector2f Pj;
		Vector2f Pij;
		Vector2f pij;
		
		for(int i=0; i<(M-1); i++){
			// Two points defining the current line in world coordinates
			Pi = lineDefs.block(i,0,1,2).transpose();
			Pj = lineDefs.block(i+1,0,1,2).transpose();

			// Vector in direction of line
			Pij = Pj-Pi;
			float Pij_norm = Pij.norm();
			//  Unit vector in direction of line
			pij = Pij/Pij_norm;
			lineDefs(i,2) = Pij_norm;
			lineDefs(i,3) = pij(0);
			lineDefs(i,4) = pij(1);

			// The angle of the line in radians
			float phi = atan2(pij(1),pij(0));
			lineDefs(i,5) = phi;

			if(i>0){
				float phim1 = lineDefs(i-1,5);
				float phiDiff = wrapToPi(phi-phim1);
				phi = phiDiff + phim1;
				// cout << "phi before=" << lineDefs(i,5) << ",after=" << phi << endl;
				lineDefs(i,5) = phi;
				lineDefs(i-1,6) = phiDiff/2;
			}
		}

		// Fix the last line
		lineDefs(M-1,2) = 10;
		lineDefs(M-1,3) = lineDefs(M-2,3); 
		lineDefs(M-1,4) = lineDefs(M-2,4); 
		lineDefs(M-1,5) = lineDefs(M-2,5); 
		lineDefs(M-1,6) = 0;

		numLines = lineDefs.rows();
		setRk(0,getPhiFixed(0));
		readyToCompute = true;
	}
}

void RobotPathFollowMPC::setRk(float d_ref, float phi_ref){
	Vector2f rk; rk << d_ref, phi_ref;
	mpc.Rk = rk.replicate(mpc.N,1);
}

Vector6f RobotPathFollowMPC::convertPose(Vector3f pose, Vector2f Pr, int i){
	Vector6f z;
	// Unpack pose vector
	// float x = pose(0);
	// float y = pose(1);
	float theta = pose(2);

	// Two points defining the current line in world coordinates
	Vector2f Pi = getPi(i);
	Vector2f Pj = getPi(i+1);

	// Ensure that varphi_ij is such that theta_err is in (-pi,pi] while maintaining theta linear.
	float phi  = getPhiFixed(i);
	float phi1 = getPhiFixed(i+1);
	// float theta_err = getThetaErr(i);
	float theta_err = wrapToPi(theta-phi);

	// Bisection angle psi
    float psi  = getPsi(i);
	// 	---------------------------------------------
	//	Compute the shortest signed distance (d) from 
	//	the robots reference point to the line:
	// 	---------------------------------------------

	//  Unit vector in direction of line
	Vector2f pij = getPij(i);

	// Front Vector (unit-normal vector to line in positive direction)...
	Vector2f fv; fv << pij(1), -pij(0);

	// Distance vector between robot reference (Pr) and line-point (Pl)
	Vector2f Pri = Pi-Pr;

	// Projection of the distance onto the line
	Vector2f proj = Pri.dot(pij)*pij;

	// Point on the line with shortest distance to the robots reference point:
	Vector2f Pl = Pi-proj;

	// Distance vector the robots reference point to the line
	Vector2f dVec = Pri-proj;

	// Signed distance from point to line
	float d = dVec.norm()*sgn(fv.dot(dVec));

	// Vector from next waypoint Pj to Pl
	Vector2f sVec = Pl-Pj;

	// Signed distance from Pl to the next waypoint Pj
	float s = sVec.norm()*sgn(sVec.dot(pij));

	z << s,d,theta_err,phi,phi1,psi;
	return z;
}

/// --- Constraints in compact form, i.e. P*u <= q
void RobotPathFollowMPC::setConstraints(float vw_min,float vw_max,float omega_min,float omega_max,float v_min,float v_max, float a_min, float a_max){
	q << vw_max, vw_max, -vw_min, -vw_min, omega_max, -omega_min, v_max, -v_min;
	acc_min = a_min;
	acc_max = a_max;
}
void RobotPathFollowMPC::setAccConstraints(float a_min, float a_max){
	acc_min = a_min;
	acc_max = a_max;
}

/// Scale the velocity to satisfy the constraints !
float RobotPathFollowMPC::scaleVelocity(float u, float v_des){
	Vector2f uBar; 
	uBar << v_des, v_des*u;
	Matrix<float,8,1> Pbar = P*uBar;
	// cout << "Matrix Pbar:\n" << Pbar << endl;

	float gam = 1;
	for(int i=0; i<8; i++){
		if(Pbar(i) > 0){
			gam = min(gam,q(i)/Pbar(i));
		}
	}
	return gam*v_des;
}


VectorXf RobotPathFollowMPC::scaleVelocityVec(VectorXf Uk, float v_des){
	int N = mpc.N;
	float vkm1 = Vk(0);
	float dVmax = min(a_des,acc_max)*T;
	float dVmin = max(-a_des,acc_min)*T;
	float dV;

	for(int n=0; n<N; n++){		
			// Satisfy the velocity constraints by saturation.
			Vk(n) = scaleVelocity(Uk(n), v_des);

			// Satisfy the acceleration constraints
			if(n == 0){ 
				dV = Vk(0)-vkm1;
				if(dV > dVmax) Vk(0) = vkm1+dVmax;
			}
			else {
				dV = Vk(n)-Vk(n-1);
				if(dV > dVmax) Vk(n) = Vk(n-1)+dVmax;
			}
	}

	// Satisfy the deceleration constraints  backward filtering
	for(int n=N-1; n>0; n--){
		dV = Vk(n) - Vk(n-1);
		if(dV < dVmin) Vk(n-1) = Vk(n)-dVmin;
	}

	// Here is a tradeoff between satisfying the velocity constraints vs. the acceleration constraints
	// ka is a mix filter
	if(Vk(0) < vkm1 + dVmin) Vk(0) = ka*(vkm1 + dVmin) + (1-ka)*Vk(0);
	// if(Vk(0) > vkm1 + dVmax) Vk(0) = vkm1 + dVmax; // Should not change anything...
	return Vk;
}

int RobotPathFollowMPC::predict(Vector6f yk, VectorXf Uk){
	int N = mpc.N;
	int kStepsFound = 0;
	int kStep = -1;
	bool val;
	float theta = pose(2);
	float s = yk(0);
	float d = yk(1);
	// float theta_err = yk(2);
	float psi = yk(5);
	float abs_psi;

	// Init for current time
	Sk(0) = s;
	Dk(0) = d;
	Thetak(0) = theta;
   	if(!isLastLine(kStepsFound)){

	// Prediction
	for(int n=0; n<N; n++){
		abs_psi = abs(psi);
		// Predicted Angular Velocities
		Omegak(n) = Vk(n)*Uk(n);

		// Predict the robot pose in controller coordinates over the horizon
		float theta_err = Thetak(n)-mpc.Rk(2*n+1);
		float cs = cos(theta_err+T*Omegak(n)/2);
		float sn = sin(theta_err+T*Omegak(n)/2);
		float TV = T*Vk(n);
		float TOmega = T*Omegak(n);

    	Sk(n+1) = Sk(n) + TV*cs - a*TOmega*sn;
    	Dk(n+1) = Dk(n) + TV*sn + a*TOmega*cs;
    	Thetak(n+1) = Thetak(n) + TOmega;

    	// Compute Tangents
		float tan1=tan(psi+PI/2);
		float tan2=tan(psi);

    	// Find kStep
	    	if(psi > 0){
	        	val = ((Dk(n) >= tan1*Sk(n) || Dk(n) <= tan2*Sk(n)) && (Sk(n) > -1)) || (Sk(n) > -ks*Vk(n)*abs_psi);
	    	}
	    	else{
	        	val = ((Dk(n) <= tan1*Sk(n) || Dk(n) >= tan2*Sk(n)) && (Sk(n) > -1)) || (Sk(n) > -ks*Vk(n)*abs_psi);
	    	}
    	

    	if(val){
    		//if(!isLastLine(kStepsFound)){
    		//	kStepsFound++;
    		
			if(++kStepsFound == 1) kStep = n;
        	Sk(n+1) = -getLineLength(kStepsFound);
        	psi = getPsi(kStepsFound); 
        	float phiVal = getPhiFixed(kStepsFound); 
    		for(int nn=n; nn<N; nn++) mpc.Rk(2*nn+1) = phiVal;
    		//}
    	}
    	}
	}
	return kStep;
}

bool RobotPathFollowMPC::isLastLine(int i){
	return currentLine+i >= numLines-2;
}

float RobotPathFollowMPC::get_s(){
	return yk(0);
}

float RobotPathFollowMPC::get_d(){
	return yk(1);
}

float RobotPathFollowMPC::get_th_err(){
	return yk(2);
}

// --- MPC Compute --- //
Vector2f RobotPathFollowMPC::compute(float x, float y, float th_new){
	if(!readyToCompute || allDone){
		ur << 0,0;
		uw << 0,0;
		return ur;
	}
	
	// Currently the internal pose requires that theta is not wrapped to [-π,π[, thus
	// it is unwrapped here:
	float th_diff = th_new-th_old;

	if(th_diff > PI){
		--th_nwrap;
	} else if(th_diff < -PI){
		++th_nwrap;
	}
	
	th_old = th_new;
	float theta = th_new+th_nwrap*TWO_PI;

	// Vector 3f pose : The robots posture in world coordinates
	pose << x, y, theta;
	
	// Vector2f Pr : The robots reference point in world coordinates.
	float cs = cos(theta);
	float sn = sin(theta);
	Pr << x + a*cs, y + a*sn;

	// Convert Pose to Controller Coordinates with respect to current line
	// yk = (s, d, th_err, phi_ij, phi_jk, psi)^T
	yk = convertPose(pose,Pr,0);

	// Clock Divider convinience method. MPC works better with slover clockrate which means longer prediction
	if(clkDiv > 1){
		if(clkTick == clkDiv) clkTick=0;
		if(clkTick++ != 0) return ur;
	}

	float s = yk(0);
	float d = yk(1);
	float theta_err = yk(2);

	// Nonlinear scaling for d
	float dScaled = d;
	if(useNLScaling && theta_err > 1e-6){
		dScaled = dScaled*sin(2*theta_err)/(2*theta_err);
	}

	// Controller State 
	Vector2f zk;
	zk << dScaled, theta;
	
	// MPC Optimized Control Gain for the Horizon
	mpc.compute(zk);
	Vk = scaleVelocityVec(mpc.Uk, v_des);
	// Uk = Lx0*zk + LR*Rk + Lum1*Uk(0);
	
	// Predict the futute postures of the robot in controller coordinates
	// and update the reference at the predicted time of the future intersections.
	kStep = predict(yk,mpc.Uk);
	// cout << "Phi:" << getPhiVec().transpose() << endl;

	float vk = Vk(0);
	float vk_min = 0.1;

	// Converge to the final point
	if(isLastLine(0)){
		if(vk > 0.5*(-s+a)){ 
			vk = 0.5*(-s+a);
			if(vk < vk_min) vk = vk_min;
		}
		if(s >= a){
			vk = 0;
			allDone = true;
		}
	}

	uk = mpc.Uk(0);
	ur << vk, uk*vk;

	timeSinceRefChange++;
	if(kStep == 0 && currentLine < numLines-2){
		currentLine++;
		timeSinceRefChange=0;
	}

	uw = invFw*ur;

	firstRun = false;
	return ur;
}

Vector2f RobotPathFollowMPC::computeW(float x, float y, float theta){
	compute(x,y,theta);
	return uw;
}

void RobotPathFollowMPC::setPose(float x, float y, float theta){
	pose << x, y, theta;
}

Vector3f RobotPathFollowMPC::getPose(){
	Vector3f wrapped_pose = pose;
	wrapped_pose(2) = wrapToPi(pose(2));
	return wrapped_pose;
}


// Print Functions for debugging
void RobotPathFollowMPC::printRobot(){
  cout << "Wheel-to-wheel distance: w = " << w << endl;
  cout << "Reference point offset from wheel-axle along xr: a = " << a << endl;
  cout << "Matrix Fw:\n"  << Fw  << endl;
  cout << "Matrix Fw^-1:\n"  << invFw  << endl;
}

void RobotPathFollowMPC::printWaypoints(){
  cout << "lineDefs:\n"  << lineDefs  << endl;
}

// --------------------------------------------------------------------------------
// ---------------------------- Helper Functions ---------------------------------- 
// --------------------------------------------------------------------------------
float RobotPathFollowMPC::wrapToPi(float angle){
	while(angle > PI) angle -= TWO_PI;
	while(angle <= -PI) angle += TWO_PI;
	return angle;
}

float RobotPathFollowMPC::thUnwrap(float th_new){
	float th_diff = th_new-th_old;
	th_old = th_new;

	if(th_diff > 1.5*PI){
		--th_nwrap;
	}
	else if(th_diff < -1.5*PI){
		++th_nwrap;
	}

	float theta = th_new + (th_nwrap * TWO_PI);
	cout << "th_nwrap=" << th_nwrap << ", th_new=" << th_new << ", th_old=" << th_old << ", th_diff=" << th_diff << endl;

	return theta;
}

/* 
	Returns the rotation matrix for rotating around the z-axis
*/
Matrix3f RobotPathFollowMPC::Rot(float angle){
	float cs = cos(angle);
	float sn = sin(angle);
	Matrix3f R; 
	R <<  cs, 	sn, 0,
		 -sn, 	cs, 0,
		  0,	0,	1;
	return R;
}

Matrix2f RobotPathFollowMPC::Rot2D(float angle){
	float cs = cos(angle);
	float sn = sin(angle);
	Matrix2f R; 
	R <<  cs, 	sn,
		 -sn, 	cs;
	return R;
}

// --------------------------------------------------------------------------------
// ---------------------------- Simulation Environment ---------------------------- 
// --------------------------------------------------------------------------------
Vector3f RobotPathFollowMPC::kinupdate(Vector3f pose, Vector2f uw, float T){
	float theta = pose(2);
	Vector2f ur = Fw*uw;
	Vector3f dxi_r;
	dxi_r << ur(0), 0, ur(1);

	// Forward Euler Approximation
	Vector3f newpose = pose + Rot(theta).transpose() * dxi_r * T;
	newpose(2) = wrapToPi(newpose(2));
	return  newpose;
}

void RobotPathFollowMPC::runSimulation(float x, float y, float theta, int nstp){
	pose << x,y,theta;
	int iter=0;
	float time=0;
	simData.resize(nstp,13);
	float T = 0.01;
	while(iter < nstp && !allDone){
		x = pose(0);
		y = pose(1);
		theta = wrapToPi(pose(2));

		ur = compute(x,y,theta);
		// uw = invFw*ur;

		simData.row(iter) << time,x,y,theta,ur(0),ur(1),uw(0),uw(1),yk(0),yk(1),yk(2), currentLine, kStep;

		// Forward Euler Approximation
		pose = kinupdate(pose,uw,T); 
		time+=T;
		iter++;
	}
	cout << "Number of steps in simulation = " << iter << endl;

	simData.conservativeResize(iter,Eigen::NoChange);
}





// ********************************************************************************************** //
//                              -----------------------------------                               //
//                              ------ Linear MPC Contrommer ------                               //
//                              -----------------------------------                               //
// ********************************************************************************************** //

/// Construct object with default parameters.
LMPC::LMPC(){
	// initSysDone = false;
	initMPCDone = false;
	condenseStateSpaceDone = false;
	designMPCDone = false;
}

LMPC::LMPC(LTIsys *argSys){
	setSys(argSys);
	initMPCDone = false;
	condenseStateSpaceDone = false;
	designMPCDone = false;
}

/// Destroy object - currently a no operation
LMPC::~LMPC(){
	// cout << "LMPC Object Was Destroyed!!" << endl;
}

void LMPC::setSys(LTIsys*argSys){
	sys = argSys;
}

void LMPC::initMPC(int arg_N){
	N = arg_N;
	int nx = sys->nx();
	int nz = sys->nz();
	int nu = sys->nu();
	int nr = sys->nr();
	int nd = sys->nd();
	Nz = N*nz;
	Nu = N*nu;
	Nr = N*nr;
	Nd = N*nd;

	// Condensed state space matrices
  	F  = MatrixXf::Zero(Nz,nx);
  	Gu = MatrixXf::Zero(Nz,Nu);
  	if(nr > 0)
  		Gr = MatrixXf::Zero(Nz,Nr);
  	else
  		Gr.resize(0,0);
  	if(nd > 0)	
  		Gd = MatrixXf::Zero(Nz,Nd);
  	else 
  		Gd.resize(0,0);

  	// Design MPC Controller
  	Qzcal   = MatrixXf::Zero(Nz,Nz);
  	Qducal  = MatrixXf::Zero(Nu,Nu);
  	Qucal   = MatrixXf::Zero(Nu,Nu);
  	Lambda  = MatrixXf::Zero(Nu,Nu);
	I0      = MatrixXf::Zero(Nu,nu);

	// Generate Lambda
	for(int i=0; i<Nu; i++){
		Lambda(i,i)   = 1;
		if(i+nu<Nu){  
		  Lambda(i+nu,i) = -1;
		}
	}

	// Generate I0
	for(int i=0; i<nu; i++){
		I0(i,i) = 1;
	}

	// Stacked Vectors
	Rk 		= VectorXf::Zero(Nr);
	Uk 		= VectorXf::Zero(Nu);

	// Set Flags
	initMPCDone = true;
}

void LMPC::condenseStateSpace(){
	int nx = sys->nx();
	int nz = sys->nz();
	int nu = sys->nu();
	int nr = sys->nr();
	int nd = sys->nd();
	MatrixXf T = sys->C;

	// Compute the first block column
	for(int i=0;i<N;i++){
	int k1 = i*nz;
	Gu.block(k1,0,nz,nu) = T*sys->Bu;
	if(sys->nr()>0) Gr.block(k1,0,nz,nr) = T*sys->Br;
	if(sys->nd()>0) Gd.block(k1,0,nz,nd) = T*sys->Bd;
	T = T*sys->A;
	F.block(k1,0,nz,nx) << T;
	}

	// Fill the rest of the matrix
	for(int i=1;i<N;i++){
	int k1 = i*nz;
	Gu.block(k1,i*nu,(N-i)*nz,nu) = Gu.block((i-1)*nz,(i-1)*nu,(N-i)*nz,nu);
	if(sys->nr()>0) Gr.block(k1,i*nr,(N-i)*nz,nr) = Gr.block((i-1)*nz,(i-1)*nr,(N-i)*nz,nr);
	if(sys->nd()>0) Gd.block(k1,i*nd,(N-i)*nz,nd) = Gd.block((i-1)*nz,(i-1)*nd,(N-i)*nz,nd);
	}
	condenseStateSpaceDone = true;
}

/// Design MPC
void LMPC::designMPC(VectorXf argQz, VectorXf argQdu, VectorXf argQu){
	Qz  = argQz;
	Qdu = argQdu;
	Qu 	= argQu;

  	// Stacked weights for quadratic program formulation.
  	Qzcal = Qz.replicate(N,1).asDiagonal();
  	Qducal = Qdu.replicate(N,1).asDiagonal();
  	Qucal = Qu.replicate(N,1).asDiagonal();
	
	// Compute the Hessian Matrix and it's inverse
	H =  Gu.transpose()*Qzcal*Gu +             // Hz
         Lambda.transpose()*Qducal*Lambda +  // Hdu
         Qucal;                              // Hu
	invH = H.inverse();

	// Linear Terms - If the optimization should be handled by 
	// qp-solver, the linear term would be gk = Mx0*zk + MR*Rk + Mum1*ukm1;
	Mx0   =  Gu.transpose()*Qzcal*F;
	if(sys->nr()>0)
		MR    =  Gu.transpose()*Qzcal*(Gr-MatrixXf::Identity(Nr,Nr));
	else 
		MR    =  Gu.transpose()*Qzcal*(-MatrixXf::Identity(Nz,Nz));

	Mum1  = -Lambda.transpose()*Qducal*I0;

	//  Controller Gains for the horizon
	Lx0  = -invH*Mx0;
	LR   = -invH*MR;
	Lum1 = -invH*Mum1;
	// Maybe compute parameters Kx0, KR, Kum1
	designMPCDone = true;
}

void LMPC::compute(VectorXf xk){
	// MPC Optimized Control Gain for the Horizon
	Uk = Lx0*xk + LR*Rk + Lum1*Uk(0);
}

void LMPC::printCondenseStateSpace(){
  cout << "Matrix F:\n"  << F  << endl;
  cout << "Matrix Gu:\n" << Gu  << endl;
  if(sys->nr()) cout << "Matrix Gr:\n" << Gr << endl;
  if(sys->nd()) cout << "Matrix Gd:\n" << Gd  << endl;
}

void LMPC::printControllerGains(){
  cout << "Matrix Lx0:\n"  << Lx0  << endl;
  cout << "Matrix LR:\n"   << LR  << endl;
  cout << "Matrix Lum1:\n" << Lum1 << endl;
}


// ********************************************************************************************** //
//                              -----------------------------------                               //
//                              ----------- LTI System ------------                               //
//                              -----------------------------------                               //
// ********************************************************************************************** //

/// Construct LTIsys object
LTIsys::LTIsys(){
}

/// Destroy object - currently a no operation
LTIsys::~LTIsys(){
	// cout << "LTIsys Object Was Destroyed!!" << endl;
}

/// Initialize System dx/dt = A*x + Bu*u, y = C*x
void LTIsys::initBuC(MatrixXf argA, MatrixXf argBu, MatrixXf argC){
	A = argA;
	Bu = argBu;
	Br.resize(0,0);
	Bd.resize(0,0);
	C = argC;
}

/// Initialize System dx/dt = A*x + Bu*u + Br*r, y = C*x
void LTIsys::initBuBrC(MatrixXf argA, MatrixXf argBu, MatrixXf argBr, MatrixXf argC){
	A = argA;
	Bu = argBu;
	Br = argBr;
	Bd.resize(0,0);
	C = argC;
}

/// Initialize System dx/dt = A*x + Bu*u + Bd*d, y = C*x
void LTIsys::initBuBdC(MatrixXf argA, MatrixXf argBu, MatrixXf argBd, MatrixXf argC){
	A = argA;
	Bu = argBu;
	Br.resize(0,0);
	Bd = argBd;
	C = argC;
}

/// Initialize System dx/dt = A*x + Bu*u + Br*r + Bd*d, y = C*x
void LTIsys::initBuBrBdC(MatrixXf argA, MatrixXf argBu, MatrixXf argBr, MatrixXf argBd, MatrixXf argC){
	A = argA;
	Bu = argBu;
	Br = argBr;
	Bd = argBd;
	C  = argC;
}

/// Return the number of states
int LTIsys::nx(){
	return A.rows();
}

/// Return the number of outputs
int LTIsys::nu(){
	return Bu.cols();
}
/// Return the number of reference inputs, same as nz usually
int LTIsys::nr(){
	return Br.cols();
}
/// Return the number of disturbance inputs
int LTIsys::nd(){
	return Bd.cols();
}
/// Return the number of controlled outputs
int LTIsys::nz(){
	return C.rows();
}

void LTIsys::printSys(){
  cout << "Matrix A:\n"  << A  << endl;
  cout << "Matrix Bu:\n"  << Bu  << endl;
  if(nr() > 0) cout << "Matrix Br:\n"  << Br  << endl;
  if(nd() > 0) cout << "Matrix Bd:\n"  << Bd  << endl;
  cout << "Matrix C:\n"  << C  << endl;
}

