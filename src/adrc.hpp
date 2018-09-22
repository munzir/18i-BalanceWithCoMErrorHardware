/**
 * @file helpers.h
 * @author Munzir
 * @date Sept 21th, 2018
 * @brief This file comtains some helper functions used for adrc balancing
 */

#include <dart/dart.hpp>

using namespace dart;
using namespace dart::dynamics;

Eigen::MatrixXd Q;
Eigen::MatrixXd R;

// read the costs Q and R
void readCosts();

// get body com
Eigen::Vector3d getBodyCOM(SkeletonPtr robot);

// compute linearized dynamics
void computeLinearizedDynamics(const SkeletonPtr robot, \
  Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::VectorXd& B_thWheel, Eigen::VectorXd& B_thCOM);
