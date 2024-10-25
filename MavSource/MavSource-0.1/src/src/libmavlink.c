#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdbool.h> 
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <time.h>

#include <libmavlink.h>

float map(float value) {
    if (value < from_min || value > from_max) { fprintf(stderr, "Error: Input value out of range\n");return 0.0;  }
    float from_range = from_max - from_min;
    float to_range = to_max - to_min;
    float scaled_value = (value - from_min) / from_range;
    float result = (scaled_value * to_range) + to_min;
    return result;
}

bool isMavlinkMessage(const uint8_t* data, size_t length) {
    if (length < MAVLINK_HEADER_LEN) { return false; }
    if (data[0] != 0xFE || data[1] != 0x09 || data[2] != MAVLINK_HEADER_LEN || data[5] != 0) { return false; }
    return true;
}
mavlink_attitude_quaternion_t createAndInitQuaternion() {
    mavlink_attitude_quaternion_t quaternion;
    quaternion.time_boot_ms = 0;
    quaternion.q1 = 0.0;
    quaternion.q2 = 0.0;
    quaternion.q3 = 0.0;
    quaternion.q4 = 0.0;
    quaternion.rollspeed = 0.0;
    quaternion.pitchspeed = 0.0 ;
    quaternion.yawspeed = 0.0 ;
    return quaternion;
}
mavlink_actuator_control_target_t createAndInitActuatorControlTarget() {
    mavlink_actuator_control_target_t actuatorControlTarget;
    actuatorControlTarget.time_usec = 0;
    for (int i = 0; i < 8; i++) {
        actuatorControlTarget.controls[i] = 0.0;
    }
    actuatorControlTarget.group_mlx = 0;
    return actuatorControlTarget;
}
mavlink_attitude_target_t createAndInitAttitudeTarget() {
    mavlink_attitude_target_t attitudeTarget;
    attitudeTarget.time_boot_ms = 0;
    attitudeTarget.q[0] = 0.0;
    attitudeTarget.q[1] = 0.0;
    attitudeTarget.q[2] = 0.0;
    attitudeTarget.q[3] = 0.0;
    attitudeTarget.body_roll_rate = 0.0;
    attitudeTarget.body_pitch_rate = 0.0;
    attitudeTarget.body_yaw_rate = 0.0;
    attitudeTarget.thrust = 0.0;
    attitudeTarget.type_mask = 0;
    return attitudeTarget;
}
states init_states(){

    states initial_state;
    initial_state.gps_raw_int_state_var = (mavlink_gps_raw_int_t){0};
    initial_state.attitude_state_var = (mavlink_attitude_t){0};
    initial_state.highres_imu_state_var = (mavlink_highres_imu_t){0};
    initial_state.global_position_int_state_var = (mavlink_global_position_int_t){0};
    initial_state.attitude_quaternion_state_var = createAndInitQuaternion();
    initial_state.local_position_ned_state_var = (mavlink_local_position_ned_t){0};
    initial_state.attitude_target_state_var = createAndInitAttitudeTarget();
    initial_state.timesync_state_var = (mavlink_timesync_t){0};
    initial_state.altitude_state_var = (mavlink_altitude_t){0};
    initial_state.actuator_control_target_state_var = createAndInitActuatorControlTarget();
    initial_state.servo_output_raw_state_var = (mavlink_servo_output_raw_t){0};
    initial_state.vfr_hud_state_var = (mavlink_vfr_hud_t){0};
    initial_state.position_target_local_ned_state_var = (mavlink_position_target_local_ned_t){0};
    initial_state.heartbeat_state_var = (mavlink_heartbeat_t){0};
    initial_state.ping_state_var = (mavlink_ping_t){0};
    initial_state.extended_sys_state_state_var = (mavlink_extended_sys_state_t){0};
    initial_state.sys_status_state_var = (mavlink_sys_status_t){0};
    initial_state.command_ack_state_var = (mavlink_command_ack_t){0};
    initial_state.estimator_status_state_var = (mavlink_estimator_status_t){0};
    initial_state.system_time_state_var = (mavlink_system_time_t){0};
    initial_state.battery_status_state_var = (mavlink_battery_status_t){0};
    initial_state.home_position_state_var = (mavlink_home_position_t){0};
    initial_state.vibration_state_var = (mavlink_vibration_t){0};
    return(initial_state);

}

void update_model(states *current_state, mavlink_message_t msg,pthread_mutex_t *state_mutex , bool HB_starter) {
    pthread_mutex_lock(state_mutex);
    if ((msg.msgid == 4 )||(msg.msgid == 245 )||(msg.msgid == 1)||(msg.msgid == 8)){ goto __end;}
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_GPS_RAW_INT:
        mavlink_msg_control_system_state_decode(&msg, &(current_state->gps_raw_int_state_var));
        break;
    case MAVLINK_MSG_ID_ATTITUDE:
        mavlink_msg_control_system_state_decode(&msg, &(current_state->attitude_state_var));
        break;
    case MAVLINK_MSG_ID_HIGHRES_IMU:
        mavlink_msg_control_system_state_decode(&msg, &(current_state->highres_imu_state_var));
        break;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        mavlink_msg_control_system_state_decode(&msg, &(current_state->global_position_int_state_var));
        break;
    case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
        mavlink_msg_control_system_state_decode(&msg, &(current_state->attitude_quaternion_state_var));
        break;
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        mavlink_msg_control_system_state_decode(&msg, &(current_state->local_position_ned_state_var));
        break;
    case MAVLINK_MSG_ID_ATTITUDE_TARGET:
        mavlink_msg_control_system_state_decode(&msg, &(current_state->attitude_target_state_var));
        break;
    case MAVLINK_MSG_ID_TIMESYNC:
        mavlink_msg_control_system_state_decode(&msg, &(current_state->timesync_state_var));
        break;
    case MAVLINK_MSG_ID_ALTITUDE:
        mavlink_msg_control_system_state_decode(&msg, &(current_state->altitude_state_var));
        break;
    case MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET:
        mavlink_msg_control_system_state_decode(&msg, &(current_state->actuator_control_target_state_var));
        break;
    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
        mavlink_msg_control_system_state_decode(&msg, &(current_state->servo_output_raw_state_var));
        break;
    case MAVLINK_MSG_ID_VFR_HUD:
        mavlink_msg_control_system_state_decode(&msg, &(current_state->vfr_hud_state_var));
        break;
    case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
        mavlink_msg_control_system_state_decode(&msg, &(current_state->position_target_local_ned_state_var));
        break;
    case MAVLINK_MSG_ID_HEARTBEAT:
        mavlink_msg_control_system_state_decode(&msg, &(current_state->heartbeat_state_var));
        if (!HB_starter){HB_starter = true ;}
        break;
    case MAVLINK_MSG_ID_PING:
        mavlink_msg_control_system_state_decode(&msg, &(current_state->ping_state_var));
        break;
    case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
        mavlink_msg_control_system_state_decode(&msg, &(current_state->extended_sys_state_state_var));
        break;
    case MAVLINK_MSG_ID_SYS_STATUS:
        mavlink_msg_control_system_state_decode(&msg, &(current_state->sys_status_state_var));
        break;
    case MAVLINK_MSG_ID_COMMAND_ACK:
        mavlink_msg_control_system_state_decode(&msg, &(current_state->command_ack_state_var));
        break;
    case MAVLINK_MSG_ID_ESTIMATOR_STATUS:
        mavlink_msg_control_system_state_decode(&msg, &(current_state->estimator_status_state_var));
        break;
    case MAVLINK_MSG_ID_SYSTEM_TIME:
        mavlink_msg_control_system_state_decode(&msg, &(current_state->system_time_state_var));
        break;
    case MAVLINK_MSG_ID_BATTERY_STATUS:
        mavlink_msg_control_system_state_decode(&msg, &(current_state->battery_status_state_var));
        break;
    case MAVLINK_MSG_ID_HOME_POSITION:
        mavlink_msg_control_system_state_decode(&msg, &(current_state->home_position_state_var));
        break;
    case MAVLINK_MSG_ID_VIBRATION:
        mavlink_msg_control_system_state_decode(&msg, &(current_state->vibration_state_var));
        break;
    default:
        pthread_mutex_unlock(&state_mutex);
        return;
    }
    __end:
    pthread_mutex_unlock(state_mutex);
}

void  initialize_controller_actions(controller_actions actions,int16_t x, int16_t y, int16_t z, int16_t r, int16_t buttons) {
    actions.mc_x = x; actions.mc_y = y; actions.mc_z = z; actions.mc_r = r; actions.mc_buttons = buttons;
}
void set_Cont_Values(controller_actions *controller, int16_t x, int16_t y, int16_t z, int16_t r, int16_t buttons) {
    controller->mc_x = x; controller->mc_y = y; controller->mc_z = z; controller->mc_r = r; controller->mc_buttons = buttons;
}
uint8_t * reverse_parse_mavlink( mavlink_message_t* message) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);
    return buffer;
}
mavlink_message_t performMotorTests(uint8_t  motor_index ,float throttle ) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(1, 200, &msg, 1, 1, MAV_CMD_DO_MOTOR_TEST, 0, motor_index, 0, throttle, 5, 1, 0, 0);
    return(msg);
}

mavlink_message_t configure_actuator_command( uint8_t actuator_index, float min, float max, float trim , motor_config * temp_config){
    mavlink_message_t msg;
    mavlink_system_t system = {1, 200, MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC};
    uint8_t system_id = system.sysid;
    uint8_t component_id = system.compid;
    mavlink_msg_command_long_pack(system_id, component_id, &msg, system_id, 
        MAV_COMP_ID_ALL, MAV_CMD_CONFIGURE_ACTUATOR, 0, temp_config->config, 0, 0, 0,temp_config->out_function, 0, 0); 
    return msg ;
}
mavlink_message_t do_set_actuator_command(uint8_t actuator_index, float *values , int actuator_hop ){
    mavlink_message_t msg;
    mavlink_system_t system = {1, 200, MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC};
    uint8_t system_id = system.sysid;
    uint8_t component_id = system.compid;
    if ((sizeof(values) / sizeof(values[0])) < 6){exit(EXIT_FAILURE);}
    if (actuator_hop > 1) {exit(EXIT_FAILURE);}
     mavlink_msg_command_long_pack(system_id, component_id, &msg, system_id,
        MAV_COMP_ID_ALL, MAV_CMD_DO_SET_ACTUATOR,0, map(values[0]), map(values[1]),
             map(values[2]), map(values[3]), map(values[4]), map(values[5]), actuator_hop);
    return msg ;
}
mavlink_message_t motor_test_command(float test_value, float timeout ,uint8_t instance_number , int motor_number){
    mavlink_message_t msg;
    mavlink_system_t system = {1, 200, MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC};
    uint8_t system_id = system.sysid;
    uint8_t component_id = system.compid;
    mavlink_msg_command_long_pack(system_id, component_id, &msg, 
        system_id, MAV_COMP_ID_ALL, MAV_CMD_ACTUATOR_TEST,0,instance_number,
            MOTOR_TEST_THROTTLE_PERCENT,map(test_value),timeout, motor_number, MOTOR_TEST_ORDER_DEFAULT , 0);
    return msg ;
}

mavlink_message_t configure_pwm_outputs(){

}

motor_config init_config(int actuator_type_ ,int config_ ,int out_function_){
    motor_config temp_config;
    if (actuator_type_ <2){ temp_config.actuator_type = actuator_type_ ;  }
    else {exit(EXIT_FAILURE);}
    if (config_ < ACTUATOR_CONFIGURATION_SPIN_DIRECTION2 +1){ temp_config.config = config_; }
    else {exit(EXIT_FAILURE);}
    if (out_function_ < ACTUATOR_OUTPUT_FUNCTION_SERVO16 +1){ temp_config.out_function = out_function_; }
    else {exit(EXIT_FAILURE);}
    temp_config.validated = false;
    return temp_config;
}
bridge_verf init_bridge(int curr_length_ , uint8_t * curr_buffer_){
    bridge_verf temp_bridge;
    if (curr_length_ >0 ){ temp_bridge.curr_length = curr_length_; }
    else {
        fprintf(stderr, "Error: Invalid argument for initialization\n");
        temp_bridge.curr_buffer = NULL;
        temp_bridge.curr_length = 0;
        return temp_bridge;
    }
    temp_bridge.curr_buffer = (uint8_t *)malloc(curr_length_ * sizeof(uint8_t));
    if (temp_bridge.curr_buffer == NULL) {
        fprintf(stderr, "Error: Memory allocation failed\n");
        temp_bridge.curr_length = 0;
        return temp_bridge;
    }
    memcpy(temp_bridge.curr_buffer, curr_buffer_ ,curr_length_);
    return temp_bridge;
}
void destroy_bridge(bridge_verf temp_bridge){
    if (&temp_bridge == NULL){
        printf("[+] NULL Bridge \n");
        exit(EXIT_FAILURE);
    }
    else {free(temp_bridge.curr_buffer); temp_bridge.curr_length = 0 ;}
}