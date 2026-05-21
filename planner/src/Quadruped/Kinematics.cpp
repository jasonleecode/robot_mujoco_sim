/*
 * Kinematics.cpp
 *
 *  Created on: 6 Jun 2024
 *      Author: Felix
 */

#include "Quadruped/Kinematics.h"
#include "Quadruped/Leg.h"
#include <cstdio>


Kinematics::Kinematics(Leg *leg): leg(leg)
{
}

Eigen::Vector3d Kinematics::getCurPosition()
{
	return getPosition(leg->getTransformation());
}

Eigen::Vector3d Kinematics::getPosition(Eigen::Matrix4d trans)
{
	Eigen::Vector3d pos;
	pos <<  trans(0, 3),
			trans(1, 3),
			trans(2, 3);
	return pos;
}


Eigen::Vector3d Kinematics::getPosition(double hipAngle,
										double thighAngle,
										double calfAngle)
{
	return getPosition(Eigen::Vector3d(hipAngle, thighAngle, calfAngle));
}

Eigen::Vector3d Kinematics::getPosition(Eigen::Vector3d angles)
{
	return getPosition(leg->getTransformation(angles));
}

Eigen::Vector3d Kinematics::jointDelta_q(Eigen::Vector3d delta_x)
{
	// Uses cached Jacobian at actual angles — callers that have qTarg angles
	// should use jointAngleCompute(curAngles, delta_x) which recomputes the
	// Jacobian at curAngles to avoid stale-Jacobian IK divergence.
	Eigen::Matrix3d J = leg->getJacobian();
	double det = J.determinant();
	Eigen::Vector3d dq;
	if (std::abs(det) < 1e-6) {
		const double lambda2 = 1e-3;
		dq = J.transpose() * (J * J.transpose() + lambda2 * Eigen::Matrix3d::Identity()).inverse() * delta_x;
	} else {
		dq = J.inverse() * delta_x;
	}
	if (!dq.allFinite()) {
		return Eigen::Vector3d::Zero();
	}
	return dq;
}

Eigen::Vector3d Kinematics::jointAngleCompute(Eigen::Vector3d curAngles,
		Eigen::Vector3d delta_x)
{
	// Always compute Jacobian at curAngles (not cached actual angles) so that
	// IK remains correct when qTarg has drifted from the actual joint angles.
	Eigen::Matrix3d J = leg->getJacobian(curAngles);
	double det = J.determinant();
	Eigen::Vector3d dq;
	if (std::abs(det) < 1e-6) {
		const double lambda2 = 1e-3;
		dq = J.transpose() * (J * J.transpose() + lambda2 * Eigen::Matrix3d::Identity()).inverse() * delta_x;
	} else {
		dq = J.inverse() * delta_x;
	}
	if (!dq.allFinite()) {
		return curAngles;
	}
	Eigen::Vector3d finalAngles = curAngles + dq;
	if (!finalAngles.allFinite()) {
		return curAngles;
	}
	leg->enforceJointLim(finalAngles);
	return finalAngles;
}

Eigen::Vector3d Kinematics::jointAngleCompute(Eigen::Vector3d delta_x)
{
    Eigen::Vector3d curAngles = leg->getAngles();
	curAngles = curAngles + jointDelta_q(delta_x);
	leg->enforceJointLim(curAngles);
	return curAngles;
}

Kinematics::~Kinematics()
{
	// TODO Auto-generated destructor stub
}

