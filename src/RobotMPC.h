/**	\file RobotMPC.h
 * \brief Linear MPC controller for Path Following Mobile Robot 
 * in the Presence of Velocity Constraints.
 *
 * Header File
 *
 * \author Hjalte Bested Møller
 * \date 6. Marts 2018	
*/

#ifndef ROBOTMPC_H
#define ROBOTMPC_H

#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

#define PI M_PI
#define TWO_PI 2*PI

/// Returns the sign of the input variable
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}	

/// Some type definitions
typedef Matrix<float,Dynamic,6>  MatrixX6f;
typedef Matrix<float,Dynamic,7>  MatrixX7f;
typedef Matrix<float,6,1>  Vector6f;
typedef Matrix<float,1,1>  Vector1f;

/*
/// Wheeled Mobile Robot Class
class WMRobot 
{	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		WMRobot();
		~WMRobot();
		Vector3f pose; //!< The robot's posture in world coordinates, i.e. \f$ \xi^w = (x,y,\theta)^T \f$, in this internal pose theta is unwrapped and continous. 
		float w;
};
*/

/// LTI system Class
class LTIsys
{	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		LTIsys();
		~LTIsys();
		void initBuC(MatrixXf A, MatrixXf Bu, MatrixXf C); //!< Initialize the system \f$ \dot{x} = Ax + B_u u, z = Cx \f$
		void initBuBrC(MatrixXf A, MatrixXf Bu, MatrixXf Br, MatrixXf C); //!< Initialize the system \f$ \dot{x} = Ax + B_u u + B_r r, z = Cx \f$
		void initBuBdC(MatrixXf A, MatrixXf Bu, MatrixXf Bd, MatrixXf C); //!< Initialize the system \f$ \dot{x} = Ax + B_u u + B_d d, z = Cx \f$
		void initBuBrBdC(MatrixXf A, MatrixXf Bu, MatrixXf Br, MatrixXf Bd, MatrixXf C); //!< Initialize the system \f$ \dot{x} = Ax + B_u u + B_r r + B_d d, z = Cx \f$

		// LTI system matrices
		MatrixXf A;		//!< The system matrix
		MatrixXf Bu;	//!< The input matrix for the controlled input \f$u\f$
		MatrixXf Br;	//!< The input matrix for the reference input \f$r\f$
		MatrixXf Bd;	//!< The input matrix for the disturbance input \f$d\f$
		MatrixXf C;		//!< The output matrix for the controlled outputs \f$z\f$
		int nx(); //!< Return the number of states in the control model
		int nu(); //!< Return the number of control inputs
		int nr(); //!< Return the number of reference inputs
		int nd(); //!< Return the number of disturbance inputs
		int nz(); //!< Return the number of controlled output

		void printSys();	//!< Print the current LTI system matrices
};


/// Linear Model Predictive Control Class
class LMPC
{	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		LMPC();
		LMPC(LTIsys * sys);
		~LMPC();
		void setSys(LTIsys * sys);
		LTIsys * sys;

		void initMPC(int N);
		int N;		//!< Control / Prediction Horizon (Samples)
		int Nx;	//!< N*nx
		int Nz;	//!< N*nz
		int Nu;	//!< N*nu
		int Nr;	//!< N*nr
		int Nd;	//!< N*nd
		bool initMPCDone;

		// Compute Condensed State Space Model
		void condenseStateSpace(); //!< Compute condensed state-space matrices \f$ F,G \f$ based on Markov Parameters
		MatrixXf F;		//!< Condense State-Space matrix.
		MatrixXf Gu;	//!< Condense State-Space matrix.
		MatrixXf Gr;	//!< Condense State-Space matrix.
		MatrixXf Gd;	//!< Condense State-Space matrix.
		bool condenseStateSpaceDone;

		// Design MPC Controller
		void designMPC(VectorXf argQz, VectorXf argQdu, VectorXf argQu);
		VectorXf Qz;	//!< Penalty on the output error \f$ |\hat{z}-\hat{r}|^2_{Q_z} \f$
		VectorXf Qdu;	//!< Penalty control rate-of-change \f$ |\Delta \hat{u}|^2_{Q_{\Delta u}} \f$
		VectorXf Qu;	//!< Penalty control control signal \f$ |\hat{u}-\bar{u}|^2_{Q_{u}} \f$

		// "Stacked" Weight Matrices
	  	MatrixXf Qzcal;		//!< Stacked weights for QP formulation of the cost function
	  	MatrixXf Qducal;	//!< Stacked weights for QP formulation of the cost function
	  	MatrixXf Qucal;		//!< Stacked weights for QP formulation of the cost function
	  	MatrixXf Lambda;	//!< Used internally to compute optimal control signal
		MatrixXf I0;		//!< Used internally to compute optimal control signal

		// Stacked Vectors
  		VectorXf Rk;	//!< Reference Trajectory for the control horizon
  		VectorXf Uk;	//!< Optimized control signal for the control horizon

		// Controller Gains
		MatrixXf H;		//!< The Hessian Matrix for the quadratic program
		MatrixXf invH;	//!< The inverse of the Hessian Matrix used for solving the quadratic program in the unconstrained case
		MatrixXf Mx0;	//!< Linear Term for the quadratic program 
		MatrixXf MR;	//!< Linear Term for the quadratic program 
		MatrixXf Mum1;	//!< Linear Term for the quadratic program 
  		MatrixXf Lx0;	//!< Unconstrained controller gain matrix for the current state
  		MatrixXf LR;	//!< Unconstrained controller gain matrix for the reference trajectory vector
  		MatrixXf Lum1;	//!< Unconstrained controller gain matrix for the previous control signal \f$u_{k-1}\f$
		bool designMPCDone;

		void printCondenseStateSpace();
		void printControllerGains();

		// Compute controll signal Uk
		void compute(VectorXf xk);
};

/// The master class for the Path-Following MPC Controller 
class RobotPathFollowMPC
{	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		RobotPathFollowMPC();
		~RobotPathFollowMPC();
		LTIsys sys;
		LMPC mpc;
		void init(float T, float v_des);
		void initRobot(float w, float a);
		bool initRobotDone;

		void initSys(int type, float T, float a, float v0, float omega0, float th_err0);
		void initMPC(int N); //!< Initialize Memory needed for MPC with the given system and horizon N

		// Functions for printing various state information
		void printRobot();  	//!< Print the Robot Parameters
		void printWaypoints(); 	//!< Print waypoints and assiciated data in format \f$(P_{i,x},P_{i,x},|P_{ij}|,p_{ij,x},p_{ij,y},\varphi_{ij},\psi)\f$ 

		int clkDiv;
		int clkTick;

		// Robot Parameters
		float w; 		//!< Wheel-to-wheel distance [m]
		float a;		//!< Reference point offset from wheel-axle along \f$x^r\f$ [m]
		Matrix2f Fw;	//!< Relation between wheelspeeds \f$u_w = (v_l,v_r)^T\f$ and robot heading and angular velocity \f$u_r = (v,\omega)^T \f$, such that \f$ u_r = F_w u_w\f$
    	Matrix2f invFw;	//!< The inverse of Fw
    	// Robot Posture in world coordinated (x,y,theta)'
    	Vector3f pose;	//!< The robot's posture in world coordinates, i.e. \f$ \xi^w = (x,y,\theta)^T \f$, in this internal pose theta is unwrapped and continous. 
    	
		// Linear State Space System
		int sysType;
		float T; 		//!< Sampling Interval [m]
		float v_des;	//!< The desired velocity [m/s]
		float a_des;	//!< The desired accleration [m/s]

	    // Controller variables
	    Vector6f yk;	//!< \f$y_k = (s_k,d_k,\tilde{\theta}_k,\varphi_{ij},\varphi_{jk},\psi_j)\f$
	    Vector2f Pr;	//!< Robot reference point in world coordinates.
	    float uk;		//!< Control input for current timestep \f$u_k\f$

		Vector2f ur;	//!< Vector with heading and angular velocity: \f$u_r = (v,\omega)^T \f$
		Vector2f uw;	//!< Vector with the left and right wheel velocities: \f$u_w = (v_l,v_r)^T\f$

		// Stacked Vectors for the prediction horizon
		VectorXf Vk;	//!< Stacked heading velocity vector for the prediction horizon
		VectorXf Omegak;//!< Stacked angular velocity vector for the prediction horizon
		VectorXf Dk;	//!< Stacked line distance vector for the prediction horizon
		VectorXf Sk;	//!< Stacked point distance vector for the prediction horizon
		VectorXf Thetak;//!< Stacked angle vector for the prediction horizon
  		// Reference Vector
  		// VectorXf Rk;	//!< Reference Trajectory for the control horizon
  		// VectorXf Uk;	//!< Optimized control signal for the control horizon

  		int kStep;		//!< The number of steps into the future where the intersection is predicted to happen. If this if zero the system starts tracking the next line.

  		void setRk(float d_ref, float phi_ref);	//!< Set the reference Rk to a constant value over the whole horizon.

		// Design Function For MPC
		void design(float Qth, float Qdu, float Qu); //!< Design MPC Controller - Compute the Gain Matrices \f$H,H^{-1},L_{x0},L_R,L_{u-1}\f$
		// float Qth; 	//!< Penalty for orientation error 
  		// float Qdu; 	//!< Controller penalty on control changes
  		// float Qu;  	//!< Controller penalty on control signal
		
		Vector2f compute(float x, float y, float theta);
		Vector2f computeW(float x, float y, float theta);
		int predict(Vector6f yk,VectorXf Uk); //!< Predict robot posture over the horizon and update the reference at meaningful time instants

		Vector6f convertPose(Vector3f pose, Vector2f Pr, int i); //!< Convert pose to controller coordinates and return \f$y_k\f$
		
		bool useNLScaling; //!< If TRUE the controller will use the nonlinear scaling of when the orientation error is big.
	
  		// Constraints (P*ur ≤ q)
  		void setConstraints(float vw_min, float vw_max, float omega_min, float omega_max, float v_min, float v_max, float acc_min, float acc_max); //!< Set Velocity Constraints and form P and q
  		void setAccConstraints(float acc_min, float acc_max);
 		float scaleVelocity(float u, float v_des); //!< Scale Velocity to satisfy constraints;
  		VectorXf scaleVelocityVec(VectorXf Uk, float v_des); //!< Scale Velocity to satisfy constraints;
  		bool useConstrains; //!< If TRUE the controller will satisfy the constraints, if FALSE the controller will ignore the constraints
  		Matrix<float, 8, 2> P; //!< Constraints in compact form: \f$ P u_r \leq q \f$
  		Matrix<float, 8, 1> q; //!< Constraints in compact form: \f$ P u_r \leq q \f$
  		Matrix<float, 8, 1> qa; //!< Constraints in compact form: \f$ P u_r \leq q \f$
  		float acc_min; //!< Minimum acceleration in meters pr. second pr. sencond
  		float acc_max; //!< Maximum acceleration in meters pr. second pr. sencond

  		// Waypoints
  		MatrixX7f lineDefs; //!< Line parameters computed from way-points in Format: \f$(P_{i,x},P_{i,x},|P_{ij}|,p_{ij,x},p_{ij,y},\varphi_{ij},\psi)\f$ 
  		int currentLine; 	//!< The index of the line currently being tracked
  		int numLines;		//!< The number of lines connecting the waypoints.. always the number of waypoints minus one.
  		void clearWaypoints(); //!< Clear Previously defined waypoint data
  		void setWaypoints(MatrixX2f waypoints); //!< Set all waypoints
  		void addWaypoint(float x, float y);		//!< Add a single waypoint
  		void insertWaypoint(float i, float x, float y); //!< Insert waypoint between waypoints at position i
  		void insertWaypointRel(float i, float x, float y); //!< Insert at currentline + i
  		void makeLineDefs(); //!< Line parameters computed from way-points in Format: \f$(P_{i,x},P_{i,x},|P_{ij}|,p_{ij,x},p_{ij,y},\varphi_{ij},\psi)\f$ 
  		bool readyToCompute;
  		bool allDone;

		// Functions for accessing the needed reference info
		Vector2f getPi(int i);		//!< Get \f$P_i\f$: Point vector 
		Vector2f getPij(int i);		//!< Get \f$p_{ij}\f$: Unit vector in direction of line
		float getPhi(int i);		//!< Get \f$\varphi_{ij}\f$: Angle of line between \f$P_i\f$ and \f$P_j\f$
		float getPhiFixed(int i);	//!< Get \f$\varphi_{ij}\f$: Fixed with respect to robot
		float getThetaErr(int i);	//!< Get \f$\tilde{\theta}_{ij} = \theta-\varphi_{ij}\f$: Angular error of robot with respect to line
		float getLineLength(int i); //!< Get \f$||P_{ij}||\f$: Unit vector in direction of line
		float getPsi(int i);		//!< Get bisection angle \f$\frac{\varphi_{jk}-\varphi_{ij}}{2}\f$:: Unit vector in direction of line
		VectorXf getPhiVec();		

  		// Helper Functions
  		float wrapToPi(float angle); //!< Wrap angle in radians to [−π π[
  		Matrix3f Rot(float angle); 	 //!< Return 3D Rotation Matrix around z-axix
  		Matrix2f Rot2D(float angle); //!< Return 2D Rotation Matrix around z-axix

  		int timeSinceRefChange;

  		// Simulation
  		void setPose(float x, float y, float theta); //!< Set Robot Pose
		Vector3f getPose(); //!< Robots Pose with theta wrapped to [−π π[

  		Vector3f kinupdate(Vector3f pose, Vector2f uw, float T);	//!< Kinematic open-loop simulation of unicycle using forward euler approximation - could be improved by proper integration
  		void runSimulation(float x, float y, float theta, int nstp); //!< Kinematic closed-loop simulation of MPC control of a unicycle started at a given pose and simulated for nstp iterations.
  		MatrixXf simData; //!< Simulation Data in Format: \f$(t,x,y,\theta,v,\omega,v_l,v_r,s,d,\tilde{\theta},L_i,k_{step})\f$ 

  		float ka; 
  		float ks; //!< The reference will be forced to change to newline if $\f$ s_k > -k_s |\psi_j| \f$

  		float th_old;
  		float th_nwrap;
  		float thUnwrap(float th_new);

  		bool isLastLine(int i);
  		float get_s();
  		float get_d();
		float get_th_err();

		bool firstRun;

};




#endif