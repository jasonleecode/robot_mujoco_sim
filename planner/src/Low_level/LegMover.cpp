/*
 * LegMover.cpp
 *
 *  Created on: 23 Jun 2024
 *      Author: Felix
 */

#include "Low_level/LegMover.h"

LegMover::LegMover(Leg *leg, std::vector<double>::iterator qTarg):
            leg(leg), leg_qTarg(qTarg)
{
    F_Ext = Eigen::Vector3d({0.0, 0.0, -15.0}); // 45N Force in the z-axis
    
}

void LegMover::moveLegStraight(Eigen::Vector3d direction, int duration, int phaseOffset)
{
    duration = std::max(duration, (1000 / PUB_RATE) * ANGLE_RES);
    countDown = 0;
    waitTime = 0;
    
    straightPhase = duration;
    swingPhase = 0;
    motionSteps = duration;
    motionStart = leg->getPosition(get_qTarg());
    targPos = motionStart + direction;
    targetZ = targPos[2];
    targetZVelocity = 0;
    motionType = STRAIGHT;
}

void LegMover::straightMover()
{
    if (straightPhase == 0) // Return when the phase is complete
    {
        countDown = 0;
        motionType = MOTION_TYPE_NUM;
        return;
    }
    if (countDown == 0)
    {
        countDown = waitTime;
    }
    else
    {
        countDown--;
        return;
    }

    // Critically damp the live height target (omega=50 rad/s). Fade its
    // motion over the last 20% of stance so a fresh IMU correction cannot
    // demand a last-tick jump or leave nonzero velocity at the next swing.
    targetZVelocity += 0.001 * (2500.0*(targetZ-targPos[2])-100.0*targetZVelocity);
    const double remaining = std::min(1.0, static_cast<double>(straightPhase)/ (0.2*motionSteps));
    const double fade = remaining*remaining*(3.0-2.0*remaining);
    targPos[2] += 0.001*fade*targetZVelocity;
    straightPhase--;
    const double t = 1.0 - static_cast<double>(straightPhase) / motionSteps;
    // Constant support speed with 10% acceleration/deceleration windows.
    const double ramp = 0.1;
    auto rampIntegral = [](double u) { return u*u*u*(1.0-0.5*u); };
    const double blend = t < ramp ? ramp*rampIntegral(t/ramp)/(1.0-ramp)
        : t > 1.0-ramp ? 1.0-ramp*rampIntegral((1.0-t)/ramp)/(1.0-ramp)
        : (t-0.5*ramp)/(1.0-ramp);
    const Eigen::Vector3d target = motionStart + blend*(targPos-motionStart);
    Eigen::Vector3d angles = get_qTarg();
    angles = leg->getKinematics()->jointAngleCompute(angles, target-leg->getPosition(angles));
    set_qTarg(leg->enforceJointLim(angles));
}

void LegMover::moveLegPosition(Eigen::Vector3d position, int duration,
                               MotionType motionType, float swingHeight, int phaseOffset)
{
    const Eigen::Vector3d direction = position - leg->getPosition(get_qTarg());
    switch (motionType)
    {
    case STRAIGHT:
        moveLegStraight(direction, duration, phaseOffset);
        break;
    case SWING:
        moveLegSwing(direction, swingHeight, duration, phaseOffset);
        break;
    
    default:
        break;
    }
}

void LegMover::mover()
{
    // std::cout << "From " << legType << std::endl;
    // std::lock_guard<std::mutex> guard(guard_mutex);

    switch (motionType)
    {
        case STRAIGHT:
            straightMover();
        break;
        case SWING:
            swingMover();
        break;
        default:
            return;
        break;
    }
}

Eigen::Vector3d LegMover::get_qTarg()
{
    Eigen::Vector3d qt(*(leg_qTarg + HIP), *(leg_qTarg + THIGH), *(leg_qTarg + CALF));
    return qt;
}

void LegMover::set_qTarg(Eigen::Vector3d angles)
{
    for (int joint = 0; joint < JOINT_NUM; ++joint)
    {
        *(leg_qTarg + joint) = angles[joint];
    }
}


void LegMover::moveLegSwing(Eigen::Vector3d direction, float swingHeight, int duration, int phaseOffset)
{
    duration = std::max(duration, (1000 / PUB_RATE) * ANGLE_RES);
    swingPhase = duration;
    motionSteps = duration;
    straightPhase = 0;
    countDown = 0;
    waitTime = 0;
    motionType = SWING;
    // Parameterize by time, not x/y: backward and vertical steps must also
    // have monotonically increasing interpolation parameters.
    motionStart = leg->getPosition(get_qTarg());
    targPos = motionStart + direction;
    swingLift = swingHeight;
}

void LegMover::swingMover()
{
    if (swingPhase == 0) {
        countDown = 0;
        motionType = MOTION_TYPE_NUM;
        return;
    }
    if (countDown > 0) {
        --countDown;
        return;
    }
    countDown = waitTime;
    --swingPhase;
    double t = 1.0 - static_cast<double>(swingPhase) / motionSteps;
    // Quintic travel and sixth-order lift: zero velocity AND acceleration
    // at both ends, including purely vertical and backward steps.
    double blend = t*t*t*(10.0 + t*(-15.0 + 6.0*t));
    Eigen::Vector3d target = motionStart + blend * (targPos - motionStart);
    target[2] += 64.0 * swingLift * t*t*t * (1.0-t)*(1.0-t)*(1.0-t);
    Eigen::Vector3d angles = get_qTarg();
    angles = leg->getKinematics()->jointAngleCompute(angles, target - leg->getPosition(angles));
    set_qTarg(leg->enforceJointLim(angles));
}
