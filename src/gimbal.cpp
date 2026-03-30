#include <gimbal.hpp>
#include <numbers>
namespace sjsu::hub {

gimbal::gimbal(hal::v5::strong_ptr<hal::actuator::rc_servo16> p_x_servo,
               hal::v5::strong_ptr<hal::actuator::rc_servo16> p_y_servo,
               hal::v5::strong_ptr<accel_source> p_accel,
               hal::v5::strong_ptr<gyro_source> p_gyro,
               hal::degrees p_min_angle,
               hal::degrees p_max_angle)
  : m_x_servo(p_x_servo)
  , m_y_servo(p_y_servo)
  , m_accel(p_accel)
  , m_gyro(p_gyro)
  , m_min_angle(p_min_angle)
  , m_max_angle(p_max_angle)
{
  float mid_angle = (m_min_angle + m_max_angle) / 2.0f;

  m_curr_x_servo_angle = mid_angle;
  m_curr_y_servo_angle = mid_angle;

  m_x_servo->position(m_curr_x_servo_angle);
  m_y_servo->position(m_curr_y_servo_angle);

  m_tar_pitch = 0.0f;
  m_filtered_pitch = 0.0f;

  m_error.eP = 0.0f;
  m_error.eI = 0.0f;
  m_error.eD = 0.0f;
}

void gimbal::set_yaw_target(hal::degrees p_yaw_deg)
{
  m_curr_x_servo_angle =
    std::clamp(p_yaw_deg, m_min_angle, m_max_angle);

  m_x_servo->position(m_curr_x_servo_angle);
}

void gimbal::set_pitch_target(hal::degrees p_pitch_deg)
{
  hal::degrees clamped_pitch =
    std::clamp(p_pitch_deg, m_min_angle, m_max_angle);

  // Convert external 0-180 pitch command into internal pitch target:
  // 0   -> -90
  // 90  ->  0
  // 180 ->  90
  m_tar_pitch = clamped_pitch - 90.0f;
}

void gimbal::update_y_servo(float p_delta_time)
{
  float alpha = m_settings.tau / (m_settings.tau + p_delta_time);

  // Reading the raw values from the sensor
   auto raw_accel = m_accel->read_acceleration();
  auto raw_gyro = m_gyro->read_gyroscope();

  // Getting the pitch from the IMU
  float angle_pitch_deg =
    atan2f(raw_accel.x,
           sqrtf(raw_accel.y * raw_accel.y + raw_accel.z * raw_accel.z)) *
    180.0f / std::numbers::pi;

  // Complementary filter
  m_filtered_pitch = alpha * (m_filtered_pitch + raw_gyro.y * p_delta_time) +
                     (1.0f - alpha) * angle_pitch_deg;

  // Error between target pitch and measured pitch
  float curr_error = m_filtered_pitch - m_tar_pitch;

  bool at_upper = (m_curr_y_servo_angle >= m_max_angle - 0.001f);
  bool at_lower = (m_curr_y_servo_angle <= m_min_angle + 0.001f);

  bool pushing_upper = at_upper && (curr_error > 0);
  bool pushing_lower = at_lower && (curr_error < 0);

  if (!pushing_upper && !pushing_lower) {
    m_error.eI += curr_error * p_delta_time;
    m_error.eI = std::clamp(m_error.eI, -200.0f, 200.0f);
  }

  m_error.eD = (curr_error - m_error.eP) / p_delta_time;
  m_error.eP = curr_error;

  // PID output is a correction, not an absolute servo angle
  float control_var = (m_settings.kP_pitch * m_error.eP) +
                      (m_settings.kI_pitch * m_error.eI) +
                      (m_settings.kD_pitch * m_error.eD);

  // Limit correction size around 0
  control_var = std::clamp(control_var, -85.0f, 85.0f);

  // Apply correction relative to center, may need to flip
  float new_angle_pos = ((m_min_angle + m_max_angle) / 2.0f) + control_var;

  // Limit how much the servo can move in one update
  new_angle_pos =
    std::clamp(new_angle_pos,
               m_curr_y_servo_angle - m_settings.max_servo_step,
               m_curr_y_servo_angle + m_settings.max_servo_step);

  new_angle_pos = std::clamp(new_angle_pos, m_min_angle, m_max_angle);
  // Updating the servo position
  m_y_servo->position(new_angle_pos);
  m_curr_y_servo_angle = new_angle_pos;
}
}  // namespace sjsu::hub
