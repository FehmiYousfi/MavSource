#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <inttypes.h>
#include <ctype.h>
#include <pthread.h>


#include <../thirdparty/c_library_v1/common/mavlink.h>
#include <../thirdparty/c_library_v1/ardupilotmega/ardupilotmega.h>
#include <../thirdparty/c_library_v1/common/mavlink_msg_rc_channels_override.h>

#define MAVLINK_HEADER_LEN 6
#define Mavlink_HeartBeat_ack 512 
#define Mavlink_HeartBeat_ack_for_real_sys 0 
#define Mavlink_mode_change_ack 176 
#define Mavlink_arm_ack 400 
#define Mavlink_ORBIT_perform_Hold_change_ack 34
#define Mavlink_Land_ack 21 

#define default_timeout 1

#define ACTUATOR_OUTPUT_FUNCTION_NONE 0
#define ACTUATOR_OUTPUT_FUNCTION_MOTOR1 1
#define ACTUATOR_OUTPUT_FUNCTION_MOTOR2 2
#define ACTUATOR_OUTPUT_FUNCTION_MOTOR3 3
#define ACTUATOR_OUTPUT_FUNCTION_MOTOR4 4
#define ACTUATOR_OUTPUT_FUNCTION_MOTOR5 5
#define ACTUATOR_OUTPUT_FUNCTION_MOTOR6 6
#define ACTUATOR_OUTPUT_FUNCTION_MOTOR7 7
#define ACTUATOR_OUTPUT_FUNCTION_MOTOR8 8
#define ACTUATOR_OUTPUT_FUNCTION_MOTOR9 9
#define ACTUATOR_OUTPUT_FUNCTION_MOTOR10 10
#define ACTUATOR_OUTPUT_FUNCTION_MOTOR11 11
#define ACTUATOR_OUTPUT_FUNCTION_MOTOR12 12
#define ACTUATOR_OUTPUT_FUNCTION_MOTOR13 13
#define ACTUATOR_OUTPUT_FUNCTION_MOTOR14 14
#define ACTUATOR_OUTPUT_FUNCTION_MOTOR15 15
#define ACTUATOR_OUTPUT_FUNCTION_MOTOR16 16
#define ACTUATOR_OUTPUT_FUNCTION_SERVO1 17
#define ACTUATOR_OUTPUT_FUNCTION_SERVO2 18
#define ACTUATOR_OUTPUT_FUNCTION_SERVO3 19
#define ACTUATOR_OUTPUT_FUNCTION_SERVO4 20
#define ACTUATOR_OUTPUT_FUNCTION_SERVO5 21
#define ACTUATOR_OUTPUT_FUNCTION_SERVO6 22
#define ACTUATOR_OUTPUT_FUNCTION_SERVO7 23
#define ACTUATOR_OUTPUT_FUNCTION_SERVO8 24
#define ACTUATOR_OUTPUT_FUNCTION_SERVO9 25
#define ACTUATOR_OUTPUT_FUNCTION_SERVO10 26
#define ACTUATOR_OUTPUT_FUNCTION_SERVO11 27
#define ACTUATOR_OUTPUT_FUNCTION_SERVO12 28
#define ACTUATOR_OUTPUT_FUNCTION_SERVO13 29
#define ACTUATOR_OUTPUT_FUNCTION_SERVO14 30
#define ACTUATOR_OUTPUT_FUNCTION_SERVO15 31
#define ACTUATOR_OUTPUT_FUNCTION_SERVO16 32

#define MOTOR_TEST_THROTTLE_PERCENT 0
#define MOTOR_TEST_THROTTLE_PWM 1
#define MOTOR_TEST_THROTTLE_PILOT 2
#define MOTOR_TEST_COMPASS_CAL 3

#define ACTUATOR_CONFIGURATION_NONE 0
#define ACTUATOR_CONFIGURATION_BEEP 1
#define ACTUATOR_CONFIGURATION_3D_MODE_ON 2
#define ACTUATOR_CONFIGURATION_3D_MODE_OFF 3
#define ACTUATOR_CONFIGURATION_SPIN_DIRECTION1 4
#define ACTUATOR_CONFIGURATION_SPIN_DIRECTION2 5

#define MOTOR_TEST_ORDER_DEFAULT 0

#define ACTUATOR_TYPE_MOTOR 0
#define ACTUATOR_TYPE_SERVO 1

#define from_min 0
#define from_max 100.0
#define to_min -1.0
#define to_max 1.0

#define arm_force_step 21196
#define temp_msg_max_size 1024

typedef struct{
    int actuator_type;
    int config;
    int out_function;
    bool validated;
} motor_config;

motor_config init_config(int actuator_type_ ,int config_ ,int out_function_);

typedef struct{
    int curr_length;
    uint8_t * curr_buffer;
}bridge_verf;

bridge_verf init_bridge(int curr_length_ , uint8_t * curr_buffer_);
void destroy_bridge(bridge_verf temp_bridge);

void extendedArrayToNormalArray(const uint8_t* extendedArray, size_t length, char* normalArray) ;
void normalArrayToExtendedArray(const char* normalArray, size_t length, uint8_t* extendedArray);
bool isMavlinkMessage(const uint8_t* data, size_t length);

typedef struct{
    int16_t mc_x;
    int16_t mc_y;
    int16_t mc_z;
    int16_t mc_r;
    int16_t mc_buttons ;
}controller_actions;

void  initialize_controller_actions(controller_actions actions,int16_t x, int16_t y, int16_t z, int16_t r, int16_t buttons);
void set_Cont_Values(controller_actions *controller, int16_t x, int16_t y, int16_t z, int16_t r, int16_t buttons);

typedef struct{
    mavlink_gps_raw_int_t gps_raw_int_state_var ;
    mavlink_attitude_t attitude_state_var ;
    mavlink_highres_imu_t highres_imu_state_var ;
    mavlink_global_position_int_t global_position_int_state_var ;
    mavlink_attitude_quaternion_t attitude_quaternion_state_var ;
    mavlink_local_position_ned_t local_position_ned_state_var ;
    mavlink_attitude_target_t attitude_target_state_var ;
    mavlink_timesync_t timesync_state_var ;
    mavlink_altitude_t altitude_state_var ;
    mavlink_actuator_control_target_t actuator_control_target_state_var ;
    mavlink_servo_output_raw_t servo_output_raw_state_var ;
    mavlink_vfr_hud_t vfr_hud_state_var ;
    mavlink_position_target_local_ned_t position_target_local_ned_state_var ;
    mavlink_heartbeat_t heartbeat_state_var ;
    mavlink_ping_t ping_state_var ;
    mavlink_extended_sys_state_t extended_sys_state_state_var ;
    mavlink_sys_status_t sys_status_state_var ;
    mavlink_command_ack_t command_ack_state_var ;
    mavlink_estimator_status_t estimator_status_state_var ;
    mavlink_system_time_t system_time_state_var ;
    mavlink_battery_status_t battery_status_state_var ;
    mavlink_home_position_t home_position_state_var ;
    mavlink_vibration_t vibration_state_var ;
}states;

states init_states();
void update_model(states *current_state,mavlink_message_t msg,pthread_mutex_t *state_mutex , bool HB_starter);
mavlink_attitude_quaternion_t createAndInitQuaternion();
mavlink_actuator_control_target_t createAndInitActuatorControlTarget();
mavlink_attitude_target_t createAndInitAttitudeTarget();
uint8_t* reverse_parse_mavlink( mavlink_message_t* message);
mavlink_message_t performMotorTests(uint8_t  motor_index ,float throttle );
uint8_t* rev_parse_msg(mavlink_message_t msg);
mavlink_message_t configure_actuator_command(uint8_t actuator_index, float min, float max, float trim ,motor_config *temp_config);
mavlink_message_t do_set_actuator_command(uint8_t actuator_index,  float* values , int actuator_hop);
mavlink_message_t motor_test_command(float test_value, float timeout ,uint8_t instance_number , int motor_number);
mavlink_message_t configure_pwm_outputs();