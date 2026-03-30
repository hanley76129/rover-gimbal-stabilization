#include <algorithm>
#include <cmath>
#include <cstdint>
#include <libhal-actuator/rc_servo.hpp>
#include <libhal-sensor/imu/icm20948.hpp>
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <limits>
#include <optional>
#include <sys/types.h>

#include <gimbal.hpp>
#include <mission_control_manager.hpp>
#include <resource_list.hpp>

#include <icm20948_sources.hpp>

namespace sjsu::hub {

using namespace hal::literals;
using namespace std::chrono_literals;

namespace {
constexpr int min_pulse_width_range = 900;
constexpr int max_pulse_width_range = 2100;

constexpr auto baudrate = 100.0_kHz;
constexpr auto gimbal_read_id = 0x300;
constexpr auto heartbeat_read_id = 0x0E;

constexpr long int16_min = std::numeric_limits<std::int16_t>::min();
constexpr long int16_max = std::numeric_limits<std::int16_t>::max();

uint8_t imu_status = 0x0;
uint8_t lcd_status = 0x0;
}  // namespace

constexpr hal::actuator::rc_servo16::settings gimbal_servo_settings{
  .frequency = 50,
  .min_angle = 0,
  .max_angle = 180,
  .min_microseconds = min_pulse_width_range,
  .max_microseconds = max_pulse_width_range,
};

int16_axis round_clamp_int16(float init_x, float init_y, float init_z)
{
  long const x_long = lroundf(init_x);
  long const y_long = lroundf(init_y);
  long const z_long = lroundf(init_z);

  return int16_axis{
    .x = static_cast<int16_t>(std::clamp<long>(x_long, int16_min, int16_max)),
    .y = static_cast<int16_t>(std::clamp<long>(y_long, int16_min, int16_max)),
    .z = static_cast<int16_t>(std::clamp<long>(z_long, int16_min, int16_max))
  };
}

void application()
{

  auto clock = resources::clock();
  auto console = resources::console();
  auto i2c = resources::i2c();
  auto mast_servo_pwm_channel_0 = resources::mast_servo_pwm_channel_0();
  auto mast_servo_pwm_channel_1 = resources::mast_servo_pwm_channel_1();
  auto can_transceiver = resources::can_transceiver();
  auto can_bus_manager = resources::can_bus_manager();
  auto can_id_filter = resources::can_identifier_filter();
  hal::print(*console, "start");
  mission_control_manager mcm(can_transceiver);
  hal::print(*console, "can initialized");

  auto icm_device = hal::v5::make_strong_ptr<hal::sensor::icm20948>(
    resources::driver_allocator(), *i2c, *clock);
  // icm_device->init_mag();
  // icm_device->auto_offsets();
  hal::print(*console, "icm initialized");
  auto gyro = hal::v5::make_strong_ptr<icm20948_gyro_source>(
    resources::driver_allocator(), icm_device);
  hal::print(*console, "s1");
  auto accel = hal::v5::make_strong_ptr<icm20948_accel_source>(
    resources::driver_allocator(), icm_device);
  hal::print(*console, "s0");

  auto mag = hal::v5::make_strong_ptr<icm20948_mag_source>(
    resources::driver_allocator(), icm_device);
  hal::print(*console, "s2");

  auto p_x_servo = hal::v5::make_strong_ptr<hal::actuator::rc_servo16>(
    resources::driver_allocator(),
    mast_servo_pwm_channel_0,
    gimbal_servo_settings);

  auto p_y_servo = hal::v5::make_strong_ptr<hal::actuator::rc_servo16>(
    resources::driver_allocator(),
    mast_servo_pwm_channel_1,
    gimbal_servo_settings);

  hal::print(*console, "servo initialized");

  gimbal mast(p_x_servo,
              p_y_servo,
              accel,
              gyro,
              gimbal_servo_settings.min_angle,
              gimbal_servo_settings.max_angle);

  can_bus_manager->baud_rate(baudrate);
  can_id_filter->allow(gimbal_read_id);
  can_id_filter->allow(heartbeat_read_id);

  auto last_time = clock->uptime();
  hal::print(*console, "starting loop initialized");
  while (true) {
    auto now_time = clock->uptime();

    // Reset heartbeat status
    imu_status = 0x0;
    lcd_status = 0x0;

    // Determine the dt of the loop
    float dt = std::chrono::duration<float>(now_time - last_time).count();
    if (dt <= 0.0f)
      dt = 1e-6f;

    // Record the time as previous
    last_time = now_time;

    // Find incoming CAN msgs
    auto gimbal_req = mcm.read_gimbal_target_request();
    if (gimbal_req) {
      hal::print(*console, "gimbal_target_requested");

      if (gimbal_req->axis == gimbal_axis::yaw) {
        mast.set_yaw_target(gimbal_req->target_angle);
      } else if (gimbal_req->axis == gimbal_axis::pitch) {
        mast.set_pitch_target(gimbal_req->target_angle);
      }
    }

    // Data to send to MC
    auto raw_accel = accel->read_acceleration();
    auto raw_gyro = gyro->read_gyroscope();
    auto raw_mag = mag->read_magnetometer();

    imu_status = 0x1;

    auto raw_accel_int16 =
      round_clamp_int16(raw_accel.x, raw_accel.y, raw_accel.z);
    auto raw_gyro_int16 = round_clamp_int16(raw_gyro.x, raw_gyro.y, raw_gyro.z);
    auto raw_mag_int16 = round_clamp_int16(raw_mag.x, raw_mag.y, raw_mag.z);

    bool hb = mcm.reply_heartbeat(imu_status, lcd_status);
    if (hb) {
      mcm.reply_imu_accel_request(raw_accel_int16);
      mcm.reply_imu_gyro_request(raw_gyro_int16);
      mcm.reply_imu_mag_request(raw_mag_int16);
      mcm.clear_heartbeat_requests();
    }

    // Update the pitch servo
    mast.update_y_servo(dt);

    hal::delay(*clock, 10ms);
  }
}
}  // namespace sjsu::hub
