#pragma once
#include <algorithm>
#include <cmath>

#include <libhal-actuator/rc_servo.hpp>    // For the two servos
#include <libhal-sensor/imu/icm20948.hpp>  // Need Gyro, Accel, and Compass
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>

#include <sensor_sources.hpp>

namespace sjsu::hub {

struct gimbal_control_settings
{
  float tau = 0.75;  // For the complementary filter

  // A PI Controller will be applied to the m_y_servo (up/down or pitch)
  float kP_pitch = 1.2f;
  float kI_pitch = 0.6f;
  float kD_pitch = 0.0f;

  // Some constants if Mission Controls ever wants to apply PID onto the
  // m_x_servo
  float kP_yaw = 0.0f;
  float kI_yaw = 0.0f;
  float kD_yaw = 0.0f;

  hal::degrees max_servo_step =
    1;  // Maximum step the servo can take during PID
};

struct pid_error
{
  float eP;  // Error on the proportional gain
  float eI;  // Error on the integral gain
  float eD;  // Error on the derivative gain
};

class gimbal
{
public:
  /**
   * @param p_x_servo the servo motor used to move the mast camera left and
   * right (no PID control on it)
   * @param p_y_servo the servo motor used to move the mast camera up and down
   * (yes PID control on it)
   * @param p_icm sensor needed to read the following: accelerometer, compass,
   * and gyroscope
   * @param p_min_angle minimum angle set inside the rc_servo_settings for both
   * the servo
   * @param p_max_angle maximum angle set inside the rc_servo_settings for both
   * the servo
   */
  gimbal(hal::v5::strong_ptr<hal::actuator::rc_servo16> p_x_servo,
         hal::v5::strong_ptr<hal::actuator::rc_servo16> p_y_servo,
         hal::v5::strong_ptr<accel_source> p_accel,
         hal::v5::strong_ptr<gyro_source> p_gyro,
         hal::degrees p_min_angle,
         hal::degrees p_max_angle);

  void set_yaw_target(hal::degrees p_yaw_deg);
  void set_pitch_target(hal::degrees p_pitch_deg);
  void update_y_servo(float p_delta_time);

private:
  hal::v5::strong_ptr<hal::actuator::rc_servo16> m_x_servo;
  hal::v5::strong_ptr<hal::actuator::rc_servo16> m_y_servo;
  hal::v5::strong_ptr<accel_source> m_accel;
  hal::v5::strong_ptr<gyro_source> m_gyro;

  hal::degrees m_min_angle, m_max_angle;

  hal::degrees m_curr_y_servo_angle, m_curr_x_servo_angle;

  float m_tar_pitch, m_curr_pitch, m_filtered_pitch;  // For PID; in degrees
  // float tar_yaw, curr_pitch; // Incase Mission Control wants PID on the yaw
  // of the gimbal

  struct pid_error m_error;
  gimbal_control_settings m_settings;
};
}  // namespace sjsu::hub
