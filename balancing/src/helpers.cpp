/**
 * @file helpers.cpp
 * @author Can Erdogan, Peng Hou
 * @date July 08, 2013
 * @brief This file contains helper functions such as imu data retrieval and -+++++++++ing...
 */

#include "helpers.h"

#include <iostream>
#include <fstream>
#include <sstream>

using namespace Eigen;
using namespace std;
using namespace Krang;

/* ******************************************************************************************** */
// Initialize the gains for controller and joystick

size_t MODE = 1;
Vector6d K_groundLo;
Vector6d K_groundHi;
Vector2d J_ground (1.0, 1.0);
Vector6d K_stand;
Vector2d J_stand;
Vector6d K_sit;
Vector2d J_sit;
Vector6d K_balLow;
Vector2d J_balLow;
Vector6d K_balHigh;
Vector2d J_balHigh;
Vector6d K = K_groundLo;

/* ******************************************************************************************** */
// Constants for the robot kinematics


const double wheelRadius = 10.5; 							///< Radius of krang wheels in inches
const double distanceBetweenWheels = 27.375; 	///< Distance Between krang's wheels in inches 

/* ******************************************************************************************** */
Eigen::MatrixXd fix (const Eigen::MatrixXd& mat) {
	Eigen::MatrixXd mat2 (mat);
	for(size_t i = 0; i < mat2.rows(); i++)
		for(size_t j = 0; j < mat2.cols(); j++)
			if(fabs(mat2(i,j)) < 1e-5) mat2(i,j) = 0.0;
	return mat2;
}

/* ******************************************************************************************** *
// Setup the indices for the motor groups

int left_arm_ids_a [7] = {11, 13, 15, 17, 19, 21, 23}; 
int right_arm_ids_a [7] = {12, 14, 16, 18, 20, 22, 24}; 
int imuWaist_ids_a [3] = {5, 8};
vector <int> left_arm_ids (left_arm_ids_a, left_arm_ids_a + 7);						
vector <int> right_arm_ids (right_arm_ids_a, right_arm_ids_a + 7);	
vector <int> imuWaist_ids (imuWaist_ids_a, imuWaist_ids_a + 2);		

// Needed to temporarily adjust the torso misaligment
int imuWaistTorso_ids_a [3] = {5, 8, 9};
vector <int> imuWaistTorso_ids (imuWaistTorso_ids_a, imuWaistTorso_ids_a + 3);		
/* ******************************************************************************************** */

/* ******************************************************************************************** */
/// Get the joint values from the encoders and the imu and compute the center of mass as well 
void getState(Vector6d& state, double dt, Vector3d* com_) {

	// Read motor encoders, imu and ft and update dart skeleton
  krang->updateSensors(dt);

	// Calculate the COM	
  Vector3d com = robot->getCOM() - robot->getPositions().segment(3,3);
//	Vector3d com = Eigen::Vector3d::Zero();
//	com(2) -= 0.264;
//	com(0) += 0.008;
//	com(0) += 0.010;
	if(com_ != NULL) *com_ = com;

	// Update the state (note for amc we are reversing the effect of the motion of the upper body)
	state(0) = atan2(com(0), com(2)); // - 0.3 * M_PI / 180.0;;
	state(1) = krang->imuSpeed;
	state(2) = (krang->amc->pos[0] + krang->amc->pos[1])/2.0 + krang->imu;
	state(3) = (krang->amc->vel[0] + krang->amc->vel[1])/2.0 + krang->imuSpeed;
	state(4) = (krang->amc->pos[1] - krang->amc->pos[0]) / 2.0;
	state(5) = (krang->amc->vel[1] - krang->amc->vel[0]) / 2.0;

	// Making adjustment in com to make it consistent with the hack above for state(0)
	com(0) = com(2) * tan(state(0));
}

/* ******************************************************************************************** */
/// Update reference left and right wheel pos/vel from joystick data where dt is last iter. time
void updateReference (double js_forw, double js_spin, double dt, Vector6d& refState) {

	// First, set the balancing angle and velocity to zeroes
	refState(0) = refState(1) = 0.0;

	// Set the distance and heading velocities using the joystick input
	refState(3) = js_forw;
	refState(5) = js_spin;

	// Integrate the reference positions with the current reference velocities
	refState(2) += dt * refState(3);
	refState(4) += dt * refState(5);
}

/* ******************************************************************************************** */
/// Returns the values of axes 1 (left up/down) and 2 (right left/right) in the joystick 
bool getJoystickInput(double& js_forw, double& js_spin) {

	// Get the message and check output is OK.
	int r = 0;
	Somatic__Joystick *js_msg = 
			SOMATIC_GET_LAST_UNPACK( r, somatic__joystick, NULL /*&protobuf_c_system_allocator*/, 4096, &js_chan );
	if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (js_msg == NULL)) return false;

	// Get the values
	for(size_t i = 0; i < 10; i++) 
		b[i] = js_msg->buttons->data[i] ? 1 : 0;
	memcpy(x, js_msg->axes->data, sizeof(x));

	// Free the joystick message
	somatic__joystick__free_unpacked(js_msg, NULL /*&protobuf_c_system_allocator*/);
	
	// Change the gains with the given joystick input
	double deltaTH = 0.2, deltaX = 0.02, deltaSpin = 0.02;
	if(!joystickControl) {
		for(size_t i = 0; i < 4; i++) {
			if(((b[5] == 0) && (b[7] == 0)) && (b[i] == 1)) K(i % 2) += ((i < 2) ? deltaTH : -deltaTH);
			else if((b[5] == 1) && (b[i] == 1)) K((i % 2) + 2) += ((i < 2) ? deltaX : -deltaX);
			else if((b[7] == 1) && (b[i] == 1)) K((i % 2) + 4) += ((i < 2) ? deltaSpin : -deltaSpin);
		}
	}

		// Reset fts
	if(!resetLeftFT && (x[4] < -0.5)) resetLeftFT = true;
	if(!resetRightFT && (x[4] > 0.5)) resetRightFT = true;
	
	// Update joystick and force-compensation controls
	static int lastb0 = b[0], lastb1 = b[1], lastb2 = b[2];
	if((b[4] == 1) && (b[6] == 0) && (b[0] == 1) && (lastb0 == 0)) { 
		joystickControl = !joystickControl;
		if(joystickControl == true) {
			somatic_motor_reset(&daemon_cx, krang->arms[LEFT]);
			somatic_motor_reset(&daemon_cx, krang->arms[RIGHT]);
		}
	}
	if((b[4] == 1) && (b[6] == 0) && (b[1] == 1) && (lastb1 == 0)) complyTorque = !complyTorque;
	if((b[4] == 1) && (b[6] == 0) && (b[2] == 1) && (lastb2 == 0)) {
		if(MODE == 4) {
			printf("Mode 5\n"); 
			K = K_balHigh;
			MODE = 5;
		}
		else if (MODE == 5) {
			printf("Mode 4\n"); 
			K = K_balLow;
			MODE = 4;
		}
	}
	lastb0 = b[0], lastb1 = b[1], lastb2 = b[2];

	// Ignore the joystick statements for the arm control 
	if((b[4] == 1) || (b[5] == 1) || (b[6] == 1) || (b[7] == 1)) {
		js_forw = js_spin = 0.0;
		return true;
	}

	// Set the values for the axis
	double* x = &(js_msg->axes->data[0]);
	if(MODE == 1 || MODE == 6) {
		js_forw = -J_ground(0) * x[1], js_spin = J_ground(1) * x[2];
	}
	else if(MODE == 4) {
		js_forw = -J_balLow(0) * x[1], js_spin = J_balLow(1) * x[2];
	}
	else if(MODE == 5) {
		js_forw = -J_balHigh(0) * x[1], js_spin = J_balHigh(1) * x[2];
	}
	else {
		js_forw = -x[1] * jsFwdAmp;
		js_spin = x[2] * jsSpinAmp;; 
	}

	
		
	return true;
}

/* ******************************************************************************************** */
/// Read file for gains
void readGains () {

	Vector6d* kgains [] = {&K_groundLo, &K_stand, &K_sit, &K_balLow, &K_balHigh, &K_groundHi};
	Vector2d* jgains [] = {&J_ground, &J_stand, &J_sit, &J_balLow, &J_balHigh, &J_ground};
	ifstream file ("../gains.txt");
	assert(file.is_open());
	char line [1024];
	for(size_t k_idx = 0; k_idx < 6; k_idx++) {
		*kgains[k_idx] = Vector6d::Zero();
		*jgains[k_idx] = Vector2d::Zero();
		file.getline(line, 1024);
		std::stringstream stream(line, std::stringstream::in);
		size_t i = 0;
		double newDouble;
		while ((i < 6) && (stream >> newDouble)) (*kgains[k_idx])(i++) = newDouble;
		while (stream >> newDouble) (*jgains[k_idx])(i++ - 6) = newDouble;
	}
	file.close();

	pv(K_groundLo);
	pv(K_groundHi);
	pv(J_ground);
	pv(K_stand);
	pv(J_stand);
	pv(K_sit);
	pv(J_sit);
	pv(K_balLow);
	pv(J_balLow);
	pv(K_balHigh);
	pv(J_balHigh);

	K = K_groundLo;
}

/* ********************************************************************************************* */
/// Sets a global variable ('start') true if the user presses 's'
void *kbhit(void *) {
	char input;
	while(true){ 
		input=cin.get(); 
		if(input=='s') start = true; 
		else if(input=='t') complyTorque = !complyTorque;
		else if(input=='e') resetLeftFT = true; 
		else if(input=='r') resetRightFT = true; 
		else if(input=='.') readGains();
		else if(input=='q') overwriteFT = true;
		else if(input=='p') spinFT = true;
		else if(input=='[') spinGoal -= 5.0;
		else if(input==']') spinGoal += 5.0;
		else if(input=='{') downGoal -= 5.0;
		else if(input=='}') downGoal += 5.0;
		else if(input=='j') { 
			joystickControl = !joystickControl;
			if(joystickControl == true) {
				somatic_motor_reset(&daemon_cx, krang->arms[LEFT]);
				somatic_motor_reset(&daemon_cx, krang->arms[RIGHT]);
			}
		}
		else if(input=='1') {
			printf("Mode 1\n"); 
			K = K_groundLo;
			MODE = 1;
		}
		else if(input=='2') {
			printf("Mode 2\n"); 
			K = K_stand;
			MODE = 2;
		}
		else if(input=='3') {
			printf("Mode 3\n"); 
			K = K_sit;
			MODE = 3;
		}
		else if(input=='4') {
			printf("Mode 4\n"); 
			K = K_balLow;
			MODE = 4;
		}
		else if(input=='5') {
			printf("Mode 5\n"); 
			K = K_balHigh;
			MODE = 5;
		}
		else if(input=='6') {
			printf("Mode 6\n"); 
			K = K_groundHi;
			MODE = 6;
		}


	}
	start = true;
}


/* ********************************************************************************************* */
/// Computes the imu value from the imu readings
void getImu (ach_channel_t* imuChan, double& _imu, double& _imuSpeed, double dt, 
		filter_kalman_t* kf) {

	// ======================================================================
	// Get the readings

	// Get the value
	int r;
	struct timespec currTime;
	clock_gettime(CLOCK_MONOTONIC, &currTime);
	struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);
	Somatic__Vector *imu_msg = SOMATIC_WAIT_LAST_UNPACK(r, somatic__vector, 
			/*&protobuf_c_system_allocator*/ NULL, IMU_CHANNEL_SIZE, imuChan, &abstime );
	assert((imu_msg != NULL) && "Imu message is faulty!");

	// Get the imu position and velocity value from the readings (note imu mounted at 45 deg).
	static const double mountAngle = -.7853981634;
	double newX = imu_msg->data[0] * cos(mountAngle) - imu_msg->data[1] * sin(mountAngle);
	_imu = atan2(newX, imu_msg->data[2]); 
	_imuSpeed = imu_msg->data[3] * sin(mountAngle) + imu_msg->data[4] * cos(mountAngle);

	// Free the unpacked message
	somatic__vector__free_unpacked( imu_msg, /*&protobuf_c_system_allocator*/ NULL );

	// ======================================================================
	// Filter the readings

	// Skip if a filter is not provided
	if(kf == NULL) return;

	// Setup the data
	kf->z[0] = _imu, kf->z[1] = _imuSpeed;

	// Setup the time-dependent process matrix
	kf->A[0] = kf->A[3] = 1.0;
	kf->A[2] = dt;

	// Setup the process noise matrix
	static const double k1 = 2.0;
	static const double k1b = 5.0;
	kf->R[0] = (dt*dt*dt*dt) * k1 * (1.0 / 4.0);
	kf->R[1] = (dt*dt*dt) * k1 * (1.0 / 2.0);
	kf->R[2] = (dt*dt*dt) * k1 * (1.0 / 2.0);
	kf->R[3] = (dt*dt) * k1b;
	
	// First make a prediction of what the reading should have been, then correct it
	filter_kalman_predict(kf);
	filter_kalman_correct(kf);

	// Set the values
	_imu = kf->x[0], _imuSpeed = kf->x[1];
}

/* ******************************************************************************************** */
/*void computeExternal (const Vector6d& input, SkeletonDynamics& robot, Vector6d& external, 
		bool left) {

	// Get the point transform wrench due to moving the affected position from com to sensor origin
	// The transform is an identity with the bottom left a skew symmetric of the point translation
	Matrix6d pTcom_sensor = MatrixXd::Identity(6,6); 
	pTcom_sensor.bottomLeftCorner<3,3>() << 0.0, -s2com(2), s2com(1), s2com(2), 0.0, -s2com(0), 
		-s2com(1), s2com(0), 0.0;

	// Get the rotation between the world frame and the sensor frame by setting the arm values
	// and the imu/waist values
	const char* nodeName = left ? "lGripper" : "rGripper";
	Matrix3d Rsw = robot.getNode(nodeName)->getWorldTransform().topLeftCorner<3,3>().transpose();
//	vector <int> dofs;
//	for(size_t i = 0; i < 25; i++) dofs.push_back(i);
//	if(myDebug) cout << "\nq in computeExternal: " << robot.getConfig(dofs).transpose() << endl;
	
	// Create the wrench with computed rotation to change the frame from the world to the sensor
	Matrix6d pSsensor_world = MatrixXd::Identity(6,6); 
	pSsensor_world.topLeftCorner<3,3>() = Rsw;
	pSsensor_world.bottomRightCorner<3,3>() = Rsw;

	// Get the weight vector (note that we use the world frame for gravity so towards -y)
	// static const double eeMass = 0.169;	// kg - ft extension
	Vector6d weightVector_in_world;
	weightVector_in_world << 0.0, 0.0, -eeMass * 9.81, 0.0, 0.0, 0.0;
	
	// Compute what the force and torque should be without any external values by multiplying the 
	// position and rotation transforms with the expected effect of the gravity 
	Vector6d wrenchWeight = pTcom_sensor * pSsensor_world * weightVector_in_world;

	// Remove the effect from the sensor value and convert the wrench into the world frame
	external = input - wrenchWeight;
	external = pSsensor_world.transpose() * external;	
}*/

/* ******************************************************************************************** */
/*void computeOffset (double imu, double waist, const somatic_motor_t& lwa, const Vector6d& raw, 
		SkeletonDynamics& robot, Vector6d& offset, bool left) {

	// Get the point transform wrench due to moving the affected position from com to sensor origin
	// The transform is an identity with the bottom left a skew symmetric of the point translation
	Matrix6d pTcom_sensor = MatrixXd::Identity(6,6); 
	pTcom_sensor.bottomLeftCorner<3,3>() << 0.0, -s2com(2), s2com(1), s2com(2), 0.0, -s2com(0), 
		-s2com(1), s2com(0), 0.0;

	// Get the rotation between the world frame and the sensor frame. 
	robot.setConfig(imuWaist_ids, Vector2d(-imu + M_PI_2, waist));
	robot.setConfig(left ? left_arm_ids : right_arm_ids, Map <Vector7d> (lwa.pos));
	const char* nodeName = left ? "lGripper" : "rGripper";
	Matrix3d R = robot.getNode(nodeName)->getWorldTransform().topLeftCorner<3,3>().transpose();
	cout << "Transform : "<< endl << R << endl;
	vector <int> dofs;
	for(size_t i = 0; i < 25; i++) dofs.push_back(i);
	cout << "\nq in computeExternal: " << robot.getConfig(dofs).transpose() << endl;

	// Create the wrench with computed rotation to change the frame from the bracket to the sensor
	Matrix6d pSsensor_bracket = MatrixXd::Identity(6,6); 
	pSsensor_bracket.topLeftCorner<3,3>() = R;
	pSsensor_bracket.bottomRightCorner<3,3>() = R;
	
	// Get the weight vector (note that we use the bracket frame for gravity so towards -y)
	Vector6d weightVector_in_bracket;
	weightVector_in_bracket << 0.0, 0.0, -eeMass * 9.81, 0.0, 0.0, 0.0;
	
	// Compute what the force and torque should be without any external values by multiplying the 
	// position and rotation transforms with the expected effect of the gravity 
	Vector6d expectedFT = pTcom_sensor * pSsensor_bracket * weightVector_in_bracket;
	pv(raw);
	pv(expectedFT);

	// Compute the difference between the actual and expected f/t values
	offset = expectedFT - raw;
	pv(offset);
}*/

/* ******************************************************************************************* */
// Compute the wrench on the wheel as an effect of the wrench acting on the sensor
/*void computeWheelWrench(const Vector6d& wrenchSensor, SkeletonDynamics& robot, Vector6d& wheelWrench, bool left) {
	
	// Get the position vector of the sensor with respect to the wheels
	const char* nodeName = left ? "lGripper" : "rGripper";
	Vector3d Tws = robot.getNode(nodeName)->getWorldTransform().topRightCorner<3,1>();
	if(0 && myDebug) cout << Tws.transpose() << endl;

	// Get the wrench shift operator to move the wrench from sensor origin to the wheel axis
	// TODO: This shifting is to the origin of the world frame in dart. It works now because it is not 
	// being updated by the amc encoders. We need to shift the wrench to a frame having origin at the 
	// wheel axis.
	Tws(2) -= 0.264;
	Matrix6d pTsensor_wheel = MatrixXd::Identity(6,6);
	pTsensor_wheel.bottomLeftCorner<3,3>() << 0.0, -Tws(2), Tws(1), Tws(2), 0.0, -Tws(0),
		-Tws(1), Tws(0), 0.0;

	// Shift the wrench from the sensor origin to the wheel axis
	wheelWrench = pTsensor_wheel * wrenchSensor;
}*/

/* ********************************************************************************************* */
/*bool getFT (somatic_d_t& daemon_cx, ach_channel_t& ft_chan, Vector6d& data) {

	// Check if there is anything to read
	int result;
	size_t numBytes = 0;
	struct timespec abstimeout = aa_tm_future(aa_tm_sec2timespec(.001));
	uint8_t* buffer = (uint8_t*) somatic_d_get(&daemon_cx, &ft_chan, &numBytes, &abstimeout, 
		ACH_O_LAST, &result);

	// Return if there is nothing to read
	if(numBytes == 0) return false;

	// Read the message with the base struct to check its type
	Somatic__BaseMsg* msg = somatic__base_msg__unpack(&(daemon_cx.pballoc), numBytes, buffer);
	if((msg->meta == NULL) || !msg->meta->has_type) return false;
	if(msg->meta->type != SOMATIC__MSG_TYPE__FORCE_MOMENT) return false;

	// Read the force-torque message and write it into the vector
	Somatic__ForceMoment* ftMessage = somatic__force_moment__unpack(&(daemon_cx.pballoc), 
		numBytes, buffer);
	for(size_t i = 0; i < 3; i++) data(i) = ftMessage->force->data[i]; 
	for(size_t i = 0; i < 3; i++) data(i+3) = ftMessage->moment->data[i]; 
	return true;
}*/

