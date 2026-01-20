#include <mujoco/mujoco.h>

#include <cstring>
#include <string>

namespace {

// ================= Plugin Data =================
struct ConstantForcePlugin {
  int body_id;
  mjtNum force[3];
};

// ================= init =================
void* init(const mjModel* m, const char* config) {
  auto* plugin = new ConstantForcePlugin();

  // 默认参数
  plugin->body_id = mj_name2id(m, mjOBJ_BODY, "box");
  plugin->force[0] = 5.0;
  plugin->force[1] = 0.0;
  plugin->force[2] = 0.0;

  return plugin;
}

// ================= compute =================
void compute(const mjModel* m, mjData* d, void* userdata) {
  auto* plugin = static_cast<ConstantForcePlugin*>(userdata);
  if (plugin->body_id < 0)
    return;

  int adr = 6 * plugin->body_id;

  // xfrc_applied: [fx fy fz tx ty tz]
  d->xfrc_applied[adr + 0] += plugin->force[0];
  d->xfrc_applied[adr + 1] += plugin->force[1];
  d->xfrc_applied[adr + 2] += plugin->force[2];
}

// ================= destroy =================
void destroy(void* userdata) {
  delete static_cast<ConstantForcePlugin*>(userdata);
}

}  // namespace

// ================= register =================
mjPLUGIN_DEFINE(constant_force_plugin, init, destroy, compute, nullptr, nullptr)