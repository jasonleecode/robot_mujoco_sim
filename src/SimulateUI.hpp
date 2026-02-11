#pragma once

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif

#include <mujoco/mujoco.h>
#include <string>

// 从simulate库提取的UI辅助函数和定义
namespace SimulateUI {

// Section IDs for left panel (UI0)
enum LeftSections {
  SECT_SIMULATION = 0,
  SECT_PHYSICS,
  SECT_RENDERING,
  SECT_VISUALIZATION,
  SECT_GAIT_ANALYSIS,
  NSECT0
};

// Section IDs for right panel (UI1)
enum RightSections {
  SECT_JOINT = 0,
  SECT_CAMERA_VIEW,
  NSECT1
};

// 标准UI定义 - Simulation Section
// 这些定义复用了simulate库的标准布局
struct SimulationSection {
  static mjuiDef* GetDefinition(int* run, double* time_scale) {
    static mjuiDef def[] = {
      {mjITEM_SECTION, "Simulation", 2, nullptr, "AS"},
      {mjITEM_RADIO, "", 2, run, "Pause\nRun"},
      {mjITEM_BUTTON, "Reset", 2, nullptr, " #259"},
      {mjITEM_SEPARATOR, "Speed", 1},
      {mjITEM_SLIDERNUM, "Scale", 2, time_scale, "0.1 2.0"},
      {mjITEM_SEPARATOR, "Motion", 1},
      {mjITEM_BUTTON, "Forward", 2, nullptr, ""},
      {mjITEM_BUTTON, "Stop", 2, nullptr, ""},
      {mjITEM_END}
    };
    return def;
  }
};

// 标准UI定义 - Physics Section
struct PhysicsSection {
  static mjuiDef* GetDefinition(int* check_gravity) {
    static mjuiDef def[] = {
      {mjITEM_SECTION, "Physics", 1, nullptr, "AP"},
      {mjITEM_CHECKINT, "Gravity", 2, check_gravity, ""},
      {mjITEM_END}
    };
    return def;
  }
};

// 标准UI定义 - Rendering Section
struct RenderingSection {
  static mjuiDef* GetDefinition(mjvOption* opt) {
    static mjuiDef def[] = {
      {mjITEM_SECTION, "Rendering", 1, nullptr, "AR"},
      {mjITEM_CHECKINT, "Wireframe", 2, &opt->frame, ""},
      {mjITEM_CHECKINT, "Shadow", 2, &opt->flags[mjRND_SHADOW], ""},
      {mjITEM_CHECKINT, "Reflection", 2, &opt->flags[mjRND_REFLECTION], ""},
      {mjITEM_END}
    };
    return def;
  }
};

// 标准UI定义 - Visualization Section
struct VisualizationSection {
  static mjuiDef* GetDefinition(mjvOption* opt) {
    static mjuiDef def[] = {
      {mjITEM_SECTION, "Visualization", 1, nullptr, "AV"},
      {mjITEM_CHECKINT, "Contact Point", 2, &opt->flags[mjVIS_CONTACTPOINT], ""},
      {mjITEM_CHECKINT, "Contact Force", 2, &opt->flags[mjVIS_CONTACTFORCE], ""},
      {mjITEM_CHECKINT, "Inertia", 2, &opt->flags[mjVIS_INERTIA], ""},
      {mjITEM_CHECKINT, "Joint", 2, &opt->flags[mjVIS_JOINT], ""},
      {mjITEM_CHECKINT, "COM", 2, &opt->flags[mjVIS_COM], ""},
      {mjITEM_END}
    };
    return def;
  }
};

// UI辅助函数 - 从simulate库提取
inline void UiModify(mjUI* ui, mjuiState* state, mjrContext* con) {
  mjui_resize(ui, con);

  // 检查并分配 Aux Buffer
  int id = ui->auxid;
  if (con->auxFBO[id] == 0 || con->auxWidth[id] != ui->width ||
      con->auxHeight[id] != ui->maxheight) {
    mjr_addAux(id, ui->width, ui->maxheight, ui->spacing.samples, con);
  }

  mjui_update(-1, -1, ui, state, con);
}

// 创建关节控制滑块
inline void AddJointSliders(mjUI* ui, mjModel* m, mjData* d) {
  static mjuiDef defSlider[] = {
    {mjITEM_SLIDERNUM, "", 2, nullptr, "-1 1"},
    {mjITEM_END}
  };

  if (!m || !d) return;

  for (int i = 0; i < m->nu && i < mjMAXUIITEM; i++) {
    defSlider[0].pdata = &d->ctrl[i];

    const char* name = mj_id2name(m, mjOBJ_ACTUATOR, i);
    if (name) {
      std::strncpy(defSlider[0].name, name, mjMAXUINAME - 1);
    } else {
      std::snprintf(defSlider[0].name, mjMAXUINAME, "Motor %d", i);
    }

    if (m->actuator_ctrllimited[i]) {
      std::snprintf(defSlider[0].other, mjMAXUITEXT, "%.4g %.4g",
                    m->actuator_ctrlrange[2 * i],
                    m->actuator_ctrlrange[2 * i + 1]);
    } else {
      std::strcpy(defSlider[0].other, "-20 20");
    }

    mjui_add(ui, defSlider);
  }
}

// 创建占位符用于图表区域
inline void AddPlaceholders(mjUI* ui, const char* section_name, int count) {
  static mjuiDef defSection[] = {
    {mjITEM_SECTION, "", 1, nullptr, ""},
    {mjITEM_END}
  };

  std::strncpy(defSection[0].name, section_name, mjMAXUINAME - 1);
  mjui_add(ui, defSection);

  static mjuiDef defPlaceholder[] = {
    {mjITEM_STATIC, ".", 2, nullptr, ""},
    {mjITEM_END}
  };

  for (int i = 0; i < count; i++) {
    mjui_add(ui, defPlaceholder);
  }
}

// 初始化标准UI主题
inline void InitializeTheme(mjUI* ui, int rectid, int auxid) {
  std::memset(ui, 0, sizeof(mjUI));
  ui->spacing = mjui_themeSpacing(0);
  ui->color = mjui_themeColor(0);
  ui->predicate = nullptr;
  ui->rectid = rectid;
  ui->auxid = auxid;
}

}  // namespace SimulateUI
