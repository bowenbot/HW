#include <cmath>

#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"

extern sp::RM_Motor motor_a;
extern sp::RM_Motor motor_b;
extern sp::Mahony imu;
extern sp::DBus remote;

sp::PID pos_pid_a(1e-3f, 15.0f, 0.2f, 0.0f, 25.0f, 5.0f, 0.5f, true, true);
sp::PID pos_pid_b(1e-3f, 25.0f, 0.2f, 0.0f, 25.0f, 5.0f, 0.5f, true, true);
sp::PID spd_pid_a(1e-3f, 0.04f, 0.002f, 0.0f, 0.08f, 0.05f, 0.2f, false, true);
sp::PID spd_pid_b(1e-3f, 0.04f, 0.002f, 0.0f, 0.08f, 0.05f, 0.2f, false, true);

volatile float g_target_a = 0.0f;
volatile float g_target_b = 0.0f;
volatile float g_target_spd_a = 0.0f;
volatile float g_target_spd_b = 0.0f;

namespace
{
float offset_a = 0.0f;
float offset_b = 0.0f;
float ratio = 1.0f;
float last_ratio = 1.0f;
bool inited = false;

float yaw_at_manual_check = 0.0f;
uint32_t last_manual_check_tick = 0;
constexpr float kManualYawWindow = 0.02f;
constexpr uint32_t kManualCheckPeriod = 100;

// 保持 0.3
constexpr float kManualThreshold = 0.3f;

// 手动锁存：检测到 A 或 B 手动转后，锁住另一方 500ms
uint32_t a_manual_lock_tick = 0;
uint32_t b_manual_lock_tick = 0;
constexpr uint32_t kManualLockMs = 500;

constexpr float kResetOffsetA = 0.0f;
constexpr float kResetOffsetB = 0.0f;
constexpr float kYawAlpha = 0.1f;
}  // namespace

extern "C" void gimbal_task()
{
  while (true) {
    switch (remote.sw_l) {
      case sp::DBusSwitchMode::DOWN:
        ratio = 0.5f;
        break;
      case sp::DBusSwitchMode::MID:
        ratio = -1.0f;
        break;
      case sp::DBusSwitchMode::UP:
        ratio = 3.0f;
        break;
    }

    static float yaw_filtered = 0.0f;
    yaw_filtered = kYawAlpha * imu.yaw + (1.0f - kYawAlpha) * yaw_filtered;
    const float yaw = yaw_filtered;

    const float a_angle = motor_a.angle;
    const float b_angle = motor_b.angle;
    const float a_speed = motor_a.speed;
    const float b_speed = motor_b.speed;

    switch (remote.sw_r) {
      case sp::DBusSwitchMode::DOWN: {
        pos_pid_a.clear();
        pos_pid_b.clear();
        spd_pid_a.clear();
        spd_pid_b.clear();
        motor_a.cmd(0.0f);
        motor_b.cmd(0.0f);
        inited = false;
        break;
      }

      case sp::DBusSwitchMode::MID: {
        if (!inited) {
          offset_a = a_angle - yaw;
          offset_b = b_angle - ratio * a_angle;
          last_ratio = ratio;
          inited = true;
          yaw_at_manual_check = yaw;
          last_manual_check_tick = HAL_GetTick();
          a_manual_lock_tick = 0;
          b_manual_lock_tick = 0;
        }

        if (ratio != last_ratio) {
          offset_b = b_angle - ratio * a_angle;
          last_ratio = ratio;
        }

        if (HAL_GetTick() - last_manual_check_tick >= kManualCheckPeriod) {
          yaw_at_manual_check = yaw;
          last_manual_check_tick = HAL_GetTick();
        }
        float yaw_change_100ms = yaw - yaw_at_manual_check;
        bool yaw_stable = std::fabs(yaw_change_100ms) < kManualYawWindow;

        float target_a_before = yaw + offset_a;
        float target_b_before = ratio * target_a_before + offset_b;

        uint32_t now = HAL_GetTick();

        // 先检测 B（如果 A 最近没被手动转过）
        bool b_manual = false;
        if (now - a_manual_lock_tick > kManualLockMs) {
          if (std::fabs(b_angle - target_b_before) > kManualThreshold && yaw_stable) {
            b_manual = true;
          }
        }

        if (b_manual) {
          float target_a_new = (b_angle - offset_b) / ratio;
          offset_a = target_a_new - yaw;
          b_manual_lock_tick = now;
        }
        else {
          // 再检测 A（如果 B 最近没被手动转过）
          if (now - b_manual_lock_tick > kManualLockMs) {
            if (std::fabs(a_angle - target_a_before) > kManualThreshold && yaw_stable) {
              offset_a = a_angle - yaw;
              a_manual_lock_tick = now;
            }
          }
        }

        // 用更新后的 offset_a 重新算目标
        float target_a = yaw + offset_a;
        float target_b = ratio * target_a + offset_b;

        pos_pid_a.calc(target_a, a_angle);
        spd_pid_a.calc(pos_pid_a.out, a_speed);
        motor_a.cmd(spd_pid_a.out);

        pos_pid_b.calc(target_b, b_angle);
        spd_pid_b.calc(pos_pid_b.out, b_speed);
        motor_b.cmd(spd_pid_b.out);

        g_target_a = target_a;
        g_target_b = target_b;
        g_target_spd_a = pos_pid_a.out;
        g_target_spd_b = pos_pid_b.out;

        break;
      }

      case sp::DBusSwitchMode::UP: {
        float target_a = yaw + kResetOffsetA;
        float target_b = yaw + kResetOffsetB;

        pos_pid_a.calc(target_a, a_angle);
        spd_pid_a.calc(pos_pid_a.out, a_speed);
        motor_a.cmd(spd_pid_a.out);

        pos_pid_b.calc(target_b, b_angle);
        spd_pid_b.calc(pos_pid_b.out, b_speed);
        motor_b.cmd(spd_pid_b.out);

        inited = false;
        break;
      }
    }

    osDelay(1);
  }
}