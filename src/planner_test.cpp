#include "planner.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
RobotState standing(double time=0) {
  RobotState s;
  s.time=time; s.qpos.assign(19,0); s.qvel.assign(18,0);
  s.qpos[2]=0.445; s.qpos[3]=1;
  for (int leg=0; leg<4; ++leg) { s.qpos[8+3*leg]=1.04; s.qpos[9+3*leg]=-1.8; }
  return s;
}
std::vector<double> walk(SpotPlanner& p, double start, double dt=0.001) {
  auto s=standing(start);
  p.setCurrentState(s); p.setMode(control::BasicMotion::kForward);
  p.setControlFrequency(dt);
  for (int i=1; i<=static_cast<int>(0.3/dt); ++i) { s.time=start+i*dt; p.update(s); }
  std::vector<double> q; p.getJointTargets(q); return q;
}

void checkFootContinuity() {
  Robot robot;
  auto* leg = robot.legs[FR];
  leg->setAngles(0, 1.04, -1.8);
  std::vector<double> q = {0, 1.04, -1.8};
  LegMover mover(leg, q.begin());
  auto position = [&]() { return leg->getPosition(Eigen::Vector3d(q[0],q[1],q[2])); };
  Eigen::Vector3d previous = position();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  const double z = previous[2];
  for (int phase=0; phase<4; ++phase) {
    const bool swing = phase%2 == 1;
    Eigen::Vector3d target = position();
    target[0] += swing ? 0.06 : -0.06;
    target[2] = z;
    mover.moveLegPosition(target, 350, swing ? SWING : STRAIGHT, 0.035f, 0);
    for (int tick=0; tick<350; ++tick) {
      // A changing attitude command, including the last milliseconds of stance.
      if (!swing) mover.setTargetZ(z + 0.005*std::sin(tick*0.1));
      mover.mover();
      Eigen::Vector3d current = position();
      Eigen::Vector3d next_velocity = (current-previous)/0.001;
      require(((next_velocity-velocity)/0.001).norm()<30.0,
              "foot acceleration spike at phase/attitude transition");
      previous = current;
      velocity = next_velocity;
    }
    require(velocity.norm()<0.01,"foot did not decelerate before phase change");
    if (swing) require((position()-target).norm()<1e-5,"swing missed its landing target");
  }
}

void checkDiagonalAlternation() {
  Robot robot;
  TrotGait gait(&robot, "");
  gait.active = true;
  gait.setStanceDuration(350);
  gait.setGaitMotion(FORWARD);
  gait.runStep();
  for (int tick=0; tick<350; ++tick) {
    if (tick==100) gait.setGaitMotion(BACKWARD);
    for (auto* mover : gait.legMovers) mover->mover();
    gait.runStep();
  }
  require(gait.legMovers[FL]->swingPhase>0 && gait.legMovers[RR]->swingPhase>0,
          "direction change repeated the same swing diagonal");
  require(gait.legMovers[FR]->straightPhase>0 && gait.legMovers[RL]->straightPhase>0,
          "direction change lost its support diagonal");
}

int main() {
  try {
    checkFootContinuity();
    checkDiagonalAlternation();
    SpotPlanner p;
    auto initial = standing();
    std::vector<double> command(initial.qpos.begin()+7, initial.qpos.end());
    for (int i=0; i<12; ++i) command[i] += 0.001*(i+1);
    p.setCurrentState(initial, command);
    std::vector<double> seeded;
    p.getJointTargets(seeded);
    require(seeded==command,"controller handoff changed the previous joint command");
    p.reset();
    auto first=walk(p,0);
    auto s=standing(0.3);
    for (int i=0; i<100; ++i) p.update(s);
    std::vector<double> q; p.getJointTargets(q);
    require(q==first,"paused simulation advanced the gait");
    p.reset(); auto after_reset=walk(p,0);
    require(after_reset==first,"reset did not clear gait/leg/timing state");
    p.reset(); auto late=walk(p,50);
    for (int i=0; i<12; ++i) require(std::abs(first[i]-late[i])<1e-9,"floating time offset changes gait rate");
    p.reset(); auto half_rate=walk(p,0,0.002);
    for (int i=0; i<12; ++i) require(std::abs(first[i]-half_rate[i])<0.015,"500Hz changes physical gait speed");
    // A rewind must restart planning immediately, not wait for old time to catch up.
    p.update(standing(0));
    p.setMode(control::BasicMotion::kForward);
    s=standing(.02); p.update(s); p.getJointTargets(q);
    require(std::abs(q[1]-1.04)>1e-6,"time rewind froze planner");
    // Fall stays latched until reset even if the caller resends movement.
    s=standing(.1); s.imu_quat={0.0,1.0,0.0,0.0}; p.update(s);
    require(p.isFallen(),"fall was not detected");
    p.setMode(control::BasicMotion::kForward); p.update(standing(.11));
    require(p.isFallen(),"movement cleared fall latch");
    p.reset(); require(!p.isFallen(),"reset did not clear fall latch");
    p.setSpeedScale(std::numeric_limits<double>::quiet_NaN());
    p.setSpeedScale(std::numeric_limits<double>::infinity());
    auto valid=walk(p,0);
    for (double v:valid) require(std::isfinite(v),"invalid speed caused invalid targets");
    bool rejected=false;
    try { p.update(RobotState{}); } catch (const std::invalid_argument&) { rejected=true; }
    require(rejected,"incomplete state was not rejected");
    puts("PASS foot continuity, diagonal alternation, planner timing, pause, rewind, reset, fall latch and invalid input");
    return 0;
  } catch (const std::exception& e) { fprintf(stderr,"FAIL: %s\n",e.what()); return 1; }
}
