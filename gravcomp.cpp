#include"gravcomp.hpp"
#include <cmath>
#include <iostream>
#include <fstream>
#include <aris.hpp>

//EE Force Compensate Caculation
//Refer to 10.16383/j.aas.2017.c150753

//p = [x, y, z, k1, k2, k3]T
//m = [mx1, my1, mz1, ... mx3, my3, mz3]T
//p = inv(F.t * F) * F.t * m


//l = [Lx, Ly, Lz, Fx0, Fy0, Fz0]T
//f = [Fx1, Fy1, Fz1, ... Fx3, Fy3, Fz3]T
//l = inv(R.t * R) * R.t * f


using namespace std;

// get torque
auto GravComp::getTorqueVector(double force_data1_[6], double force_data2_[6], double force_data3_[6], double torque_vector_[9]) -> void {
	std::copy(force_data1_ + 3, force_data1_ + 6, torque_vector_);
	std::copy(force_data2_ + 3, force_data2_ + 6, torque_vector_ + 3);
	std::copy(force_data3_ + 3, force_data3_ + 6, torque_vector_ + 6);
}

// get force
auto GravComp::getForceVector(double force_data1_[6], double force_data2_[6], double force_data3_[6], double force_vector_[9]) -> void {
	std::copy(force_data1_, force_data1_ + 3, force_vector_);
	std::copy(force_data2_, force_data2_ + 3, force_vector_ + 3);
	std::copy(force_data3_, force_data3_ + 3, force_vector_ + 6);
}

// temp function
auto GravComp::getTempFMatrix(double force_data_[6], double temp_[18]) -> void {
	double tempInit[18] = { 0, force_data_[2], -force_data_[1], 1, 0, 0,
							-force_data_[2], 0, force_data_[0], 0, 1, 0,
							force_data_[1], -force_data_[0], 0, 0, 0, 1 };
	std::copy(tempInit, tempInit + 18, temp_);
}

// temp function
auto GravComp::getTempRMatrix(double pose_matrix_[9], double temp_[18]) -> void {
	double tempInit[18] = { pose_matrix_[0], pose_matrix_[3], pose_matrix_[6], 1, 0, 0,
							pose_matrix_[1], pose_matrix_[4], pose_matrix_[7], 0, 1, 0,
							pose_matrix_[2], pose_matrix_[5], pose_matrix_[8], 0, 0, 1 };
	std::copy(tempInit, tempInit + 18, temp_);
}

auto GravComp::getInverseRm(double rotation_matrix_[9], double inverse_rot_[9]) -> void {
    double tempInit[9] = { rotation_matrix_[0], rotation_matrix_[3], rotation_matrix_[6],
                            rotation_matrix_[1], rotation_matrix_[4], rotation_matrix_[7],
                            rotation_matrix_[2], rotation_matrix_[5], rotation_matrix_[8], };
    std::copy(tempInit, tempInit + 9, inverse_rot_);
}


// get F 
auto GravComp::getFMatrix(double force_data1_[6], double force_data2_[6], double force_data3_[6], double f_matrix_[54]) -> void {
	double temp1[18] = { 0 };
	double temp2[18] = { 0 };
	double temp3[18] = { 0 };

	getTempFMatrix(force_data1_, temp1);
	getTempFMatrix(force_data2_, temp2);
	getTempFMatrix(force_data3_, temp3);

	std::copy(temp1, temp1 + 18, f_matrix_);
	std::copy(temp2, temp2 + 18, f_matrix_ + 18);
	std::copy(temp3, temp3 + 18, f_matrix_ + 36);
}

// get R
auto GravComp::getRMatrix(double pose_matrix1_[9], double pose_matrix2_[9], double pose_matrix3_[9], double r_matrix_[54]) -> void {
	double temp1[18] = { 0 };
	double temp2[18] = { 0 };
	double temp3[18] = { 0 };

	getTempRMatrix(pose_matrix1_, temp1);
	getTempRMatrix(pose_matrix2_, temp2);
	getTempRMatrix(pose_matrix3_, temp3);

	std::copy(temp1, temp1 + 18, r_matrix_);
	std::copy(temp2, temp2 + 18, r_matrix_ + 18);
	std::copy(temp3, temp3 + 18, r_matrix_ + 36);
}

// get P and L vector, Least Square
auto GravComp::getPLMatrix(double f_r_matrix_[54], double torque_force_data_[9], double P_L[6]) -> void {
	double U[54]{ 0 };
	double Inv_[54]{ 0 };
	double tau[9]{ 0 };
    double tau2[9]{0};

	aris::Size p[9];
	aris::Size rank;

	//   m : line  n : column 
	//   rank:no need
	//   U :        m x n
	//  tau : max(m,n) x 1
	//    p : max(m,n) x 1
	//    x :        n x m

	// refer to math_matrix.hpp in aris



	// utp
	aris::dynamic::s_householder_utp(9, 6, f_r_matrix_, U, tau, p, rank, 1e-6);
	// inverse
    aris::dynamic::s_householder_up2pinv(9, 6, rank, U, tau, p, Inv_, tau2, 1e-6);
//	aris::dynamic::s_householder_up2pinv(9, 6, rank, U, p, Inv_, 1e-6);
	// multiple
	aris::dynamic::s_mm(6, 1, 9, Inv_, torque_force_data_, P_L);
}

// get compensated force
auto GravComp::getCompFT(double current_pose_[16], double L_[6], double P_[6], double comp_f_[6]) -> void {
	double current_rotate[9]{ 0 };
	double inv_rotate[9]{ 0 };

	double G_vector[3]{ 0 };
	double F_vector[3]{ 0 };
	double L_vector[3]{ 0 };
	double Mass_center[3]{ 0 };
	double K_vector[3]{ 0 };

	double Mg[3]{ 0 };
	double M0[3]{ 0 };

	std::copy(L_ + 3, L_ + 6, F_vector);  // fx0 fy0 fz0
    //aris::dynamic::dsp(1, 3, F_vector);
	std::copy(L_, L_ + 3, L_vector);  // lx ly lz -> Gx Gy Gz
	std::copy(P_, P_ + 3, Mass_center);  // x0 y0 z0 -> Mgx Mgy Mgz
	std::copy(P_ + 3, P_ + 6, K_vector);  // k1 k2 k3 -> Mx0 My0 Mz0

	aris::dynamic::s_pm2rm(current_pose_, current_rotate);
    getInverseRm(current_rotate, inv_rotate);

    aris::dynamic::s_mm(3,1,3,inv_rotate,L_vector,G_vector);
    //aris::dynamic::dsp(1, 3, G_vector);

	// comp x y z
	for (int i = 0; i < 3; i++) {
		comp_f_[i] = -F_vector[i] - G_vector[i];
	}

	Mg[0] = -G_vector[2] * Mass_center[1] - G_vector[1] * Mass_center[2];
	Mg[1] = -G_vector[0] * Mass_center[2] - G_vector[2] * Mass_center[0];
	Mg[2] = -G_vector[1] * Mass_center[0] - G_vector[0] * Mass_center[1];

	M0[0] = K_vector[0] - F_vector[1] * Mass_center[2] - F_vector[2] * Mass_center[1];
	M0[1] = K_vector[1] - F_vector[2] * Mass_center[0] - F_vector[1] * Mass_center[2];
	M0[2] = K_vector[2] - F_vector[0] * Mass_center[1] - F_vector[0] * Mass_center[0];

	// comp mx my mz
	for (int i = 3; i < 6; i++) {
		comp_f_[i] = -M0[i - 3] - Mg[i - 3];
	}
}


auto GravComp::savePLVector(const double P1_[6], const double L1_[6], const double P2_[6], const double L2_[6]) -> void {

	// Get Path
	auto fspath = std::filesystem::absolute(".");
    const std::string fsfile = "force_vector.txt";
	fspath = fspath / fsfile;

	// Open file
    ofstream outFile(fspath);
	if (!outFile) {
		cerr << "Cannot Write Target File" << endl;
		return;
	}

	// Write P1 Vector
	for (int i = 0; i < 6; ++i) {
		outFile << P1_[i] << " ";
	}
	outFile << endl;

	
	// Write L1 Vector
	for (int i = 0; i < 6; ++i) {
		outFile << L1_[i] << " ";
	}
	outFile << endl;

	// Write P2 Vector
	for (int i = 0; i < 6; ++i) {
		outFile << P2_[i] << " ";
	}
	outFile << endl;

	// Write l2 Vector
	for (int i = 0; i < 6; ++i) {
		outFile << L2_[i] << " ";
	}
	outFile << endl;

	outFile.close();


}


auto GravComp::loadPLVector(double P1_[6], double L1_[6], double P2_[6], double L2_[6]) -> void {

	// Get Path
	auto fspath = std::filesystem::absolute(".");
    const std::string fsfile = "force_vector.txt";
	fspath = fspath / fsfile;

	// Open file
	ifstream inFile(fspath);
	if (!inFile) {
		cerr << "Cannot Read Target File" << endl;
		return;
	}

	// Read P Vector
	for (int i = 0; i < 6; ++i) {
		inFile >> P1_[i];
	}

	// Read L1 Vector
	for (int i = 0; i < 6; ++i) {
		inFile >> L1_[i];
	}

	// Read P2 Vector
	for (int i = 0; i < 6; ++i) {
		inFile >> P2_[i];
	}

	// Read L2 Vector
	for (int i = 0; i < 6; ++i) {
		inFile >> L2_[i];
	}

	inFile.close();


}


auto GravComp::saveInitForce(const double arm1_init_force_[6], const double arm2_init_force_[6]) -> void {

	// Get Path
	auto fspath = std::filesystem::absolute(".");
	const std::string fsfile = "init_force.txt";
	fspath = fspath / fsfile;

	// Open file
	ofstream outFile(fspath);
	if (!outFile) {
		cerr << "Cannot Write Target File" << endl;
		return;
	}


	for (int i = 0; i < 6; ++i) {
		outFile << arm1_init_force_[i] << " ";
	}
	outFile << endl;

	for (int i = 0; i < 6; ++i) {
		outFile << arm2_init_force_[i] << " ";
	}
	outFile << endl;



	outFile.close();


}




auto GravComp::loadInitForce(double arm1_init_force_[6], double arm2_init_force_[6]) -> void {

	// Get Path
	auto fspath = std::filesystem::absolute(".");
	const std::string fsfile = "init_force.txt";
	fspath = fspath / fsfile;

	// Open file
	ifstream inFile(fspath);
	if (!inFile) {
		cerr << "Cannot Read Target File" << endl;
		return;
	}

	for (int i = 0; i < 6; ++i) {
		inFile >> arm1_init_force_[i];
	}

	for (int i = 0; i < 6; ++i) {
		inFile >> arm2_init_force_[i];
	}

	inFile.close();


}

