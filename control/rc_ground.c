/*
 *   This file is part of DroneBridge: https://github.com/seeul8er/DroneBridge
 *
 *   Copyright 2017 Wolfgang Christl
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 */

#include <stdint.h>
#include <unistd.h>
#include "../common/db_raw_send_receive.h"
#include "../common/db_crc.h"
#include "../common/shared_memory.h"
#include "../common/mavlink/c_library_v2/common/mavlink.h"
#include "parameter.h"
#include "../common/db_common.h"


int rc_protocol;
uint8_t crc_mspv2, crc8, rc_seq_number = 0;
crc_t crc_rc;
int i_crc, i_rc, num_interfaces = 0;
unsigned int rc_crc_tbl_idx, mspv2_tbl_idx;
db_rc_values_t *shm_rc_values = NULL;
db_rc_overwrite_values_t *shm_rc_overwrite = NULL;
struct timespec timestamp;
bool en_rc_overwrite = false;

// pointing right into the sockets send buffer for max performance
struct data_uni *monitor_databuffer;

// DroneBridge raw interfaces. One per adapter
db_socket_t raw_interfaces_rc[DB_MAX_ADAPTERS] = {0};

// could do this with two for-loops but hardcoded is faster and number of aux channels won't change anyways
void generate_msp(unsigned short *newJoystickData) {
    monitor_databuffer->bytes[0] = 0x24;
    monitor_databuffer->bytes[1] = 0x4d;
    monitor_databuffer->bytes[2] = 0x3c;
    monitor_databuffer->bytes[3] = 0x1c;
    monitor_databuffer->bytes[4] = 0xc8;

    // in  (opentx) : AETR1234
    // out (msp)    : AERT12345678 = RollPitchYawThrottle

    //Roll
    monitor_databuffer->bytes[5] = (uint8_t) (newJoystickData[0] & 0xFF);
    monitor_databuffer->bytes[6] = (uint8_t) ((newJoystickData[0] >> 8) & 0xFF);
    //Pitch
    monitor_databuffer->bytes[7] = (uint8_t) (newJoystickData[1] & 0xFF);
    monitor_databuffer->bytes[8] = (uint8_t) ((newJoystickData[1] >> 8) & 0xFF);
    //Yaw
    monitor_databuffer->bytes[9] = (uint8_t) (newJoystickData[3] & 0xFF);
    monitor_databuffer->bytes[10] = (uint8_t) ((newJoystickData[3] >> 8) & 0xFF);
    //Throttle
    monitor_databuffer->bytes[11] = (uint8_t) (newJoystickData[2] & 0xFF);
    monitor_databuffer->bytes[12] = (uint8_t) ((newJoystickData[2] >> 8) & 0xFF);
    //AUX 1
    monitor_databuffer->bytes[13] = (uint8_t) (newJoystickData[4] & 0xFF);
    monitor_databuffer->bytes[14] = (uint8_t) ((newJoystickData[4] >> 8) & 0xFF);
    //AUX 2
    monitor_databuffer->bytes[15] = (uint8_t) (newJoystickData[5] & 0xFF);
    monitor_databuffer->bytes[16] = (uint8_t) ((newJoystickData[5] >> 8) & 0xFF);
    //AUX 3
    monitor_databuffer->bytes[17] = (uint8_t) (newJoystickData[6] & 0xFF);
    monitor_databuffer->bytes[18] = (uint8_t) ((newJoystickData[6] >> 8) & 0xFF);
    //AUX 4
    monitor_databuffer->bytes[19] = (uint8_t) (newJoystickData[7] & 0xFF);
    monitor_databuffer->bytes[20] = (uint8_t) ((newJoystickData[7] >> 8) & 0xFF);
    //AUX 5
    monitor_databuffer->bytes[21] = (uint8_t) (newJoystickData[8] & 0xFF);
    monitor_databuffer->bytes[22] = (uint8_t) ((newJoystickData[8] >> 8) & 0xFF);
    //AUX 6
    monitor_databuffer->bytes[23] = (uint8_t) (newJoystickData[9] & 0xFF);
    monitor_databuffer->bytes[24] = (uint8_t) ((newJoystickData[9] >> 8) & 0xFF);
    //AUX 7
    monitor_databuffer->bytes[25] = (uint8_t) (newJoystickData[10] & 0xFF);
    monitor_databuffer->bytes[26] = (uint8_t) ((newJoystickData[10] >> 8) & 0xFF);
    //AUX 8
    monitor_databuffer->bytes[27] = (uint8_t) (newJoystickData[11] & 0xFF);
    monitor_databuffer->bytes[28] = (uint8_t) ((newJoystickData[11] >> 8) & 0xFF);
    //AUX 9
    monitor_databuffer->bytes[29] = (uint8_t) (newJoystickData[12] & 0xFF);
    monitor_databuffer->bytes[30] = (uint8_t) ((newJoystickData[12] >> 8) & 0xFF);
    //AUX 10
    monitor_databuffer->bytes[31] = (uint8_t) (newJoystickData[13] & 0xFF);
    monitor_databuffer->bytes[32] = (uint8_t) ((newJoystickData[13] >> 8) & 0xFF);
    // CRC
    crc8 = 0; // not really a crc more a checksum
    for (int i = 3; i < 33; i++) {
        crc8 ^= (monitor_databuffer->bytes[i] & 0xFF);
    }
    monitor_databuffer->bytes[33] = crc8;
}

void generate_mspv2(unsigned short *newJoystickData) {
    monitor_databuffer->bytes[0] = 0x24;
    monitor_databuffer->bytes[1] = 0x58;
    monitor_databuffer->bytes[2] = 0x3c;
    monitor_databuffer->bytes[3] = 0x00; // flag
    monitor_databuffer->bytes[4] = 0xc8; // function
    monitor_databuffer->bytes[5] = 0x00; // function
    monitor_databuffer->bytes[6] = 0x1c; // payload size
    monitor_databuffer->bytes[7] = 0x00; // payload size

    // in  (opentx) : AETR1234
    // out (msp)    : AERT12345678 = RollPitchYawThrottle

    //Roll
    monitor_databuffer->bytes[8] = (uint8_t) (newJoystickData[0] & 0xFF);
    monitor_databuffer->bytes[9] = (uint8_t) ((newJoystickData[0] >> 8) & 0xFF);
    //Pitch
    monitor_databuffer->bytes[10] = (uint8_t) (newJoystickData[1] & 0xFF);
    monitor_databuffer->bytes[11] = (uint8_t) ((newJoystickData[1] >> 8) & 0xFF);
    //Yaw
    monitor_databuffer->bytes[12] = (uint8_t) (newJoystickData[3] & 0xFF);
    monitor_databuffer->bytes[13] = (uint8_t) ((newJoystickData[3] >> 8) & 0xFF);
    //Throttle
    monitor_databuffer->bytes[14] = (uint8_t) (newJoystickData[2] & 0xFF);
    monitor_databuffer->bytes[15] = (uint8_t) ((newJoystickData[2] >> 8) & 0xFF);
    //AUX 1
    monitor_databuffer->bytes[16] = (uint8_t) (newJoystickData[4] & 0xFF);
    monitor_databuffer->bytes[17] = (uint8_t) ((newJoystickData[4] >> 8) & 0xFF);
    //AUX 2
    monitor_databuffer->bytes[18] = (uint8_t) (newJoystickData[5] & 0xFF);
    monitor_databuffer->bytes[19] = (uint8_t) ((newJoystickData[5] >> 8) & 0xFF);
    //AUX 3
    monitor_databuffer->bytes[20] = (uint8_t) (newJoystickData[6] & 0xFF);
    monitor_databuffer->bytes[21] = (uint8_t) ((newJoystickData[6] >> 8) & 0xFF);
    //AUX 4
    monitor_databuffer->bytes[22] = (uint8_t) (newJoystickData[7] & 0xFF);
    monitor_databuffer->bytes[23] = (uint8_t) ((newJoystickData[7] >> 8) & 0xFF);
    //AUX 5
    monitor_databuffer->bytes[24] = (uint8_t) (newJoystickData[8] & 0xFF);
    monitor_databuffer->bytes[25] = (uint8_t) ((newJoystickData[8] >> 8) & 0xFF);
    //AUX 6
    monitor_databuffer->bytes[26] = (uint8_t) (newJoystickData[9] & 0xFF);
    monitor_databuffer->bytes[27] = (uint8_t) ((newJoystickData[9] >> 8) & 0xFF);
    //AUX 7
    monitor_databuffer->bytes[28] = (uint8_t) (newJoystickData[10] & 0xFF);
    monitor_databuffer->bytes[29] = (uint8_t) ((newJoystickData[10] >> 8) & 0xFF);
    //AUX 8
    monitor_databuffer->bytes[30] = (uint8_t) (newJoystickData[11] & 0xFF);
    monitor_databuffer->bytes[31] = (uint8_t) ((newJoystickData[11] >> 8) & 0xFF);
    //AUX 9
    monitor_databuffer->bytes[32] = (uint8_t) (newJoystickData[12] & 0xFF);
    monitor_databuffer->bytes[33] = (uint8_t) ((newJoystickData[12] >> 8) & 0xFF);
    //AUX 10
    monitor_databuffer->bytes[34] = (uint8_t) (newJoystickData[13] & 0xFF);
    monitor_databuffer->bytes[35] = (uint8_t) ((newJoystickData[13] >> 8) & 0xFF);
    // CRC
    crc_mspv2 = 0;
    for (int i = 3; i < 36; i++) {
        mspv2_tbl_idx = crc_mspv2 ^ monitor_databuffer->bytes[i];
        crc_mspv2 = crc_dvb_s2_table[mspv2_tbl_idx] & 0xff;
    }
    monitor_databuffer->bytes[36] = crc_mspv2;
}


uint16_t generate_mavlinkv2_rc_overwrite(unsigned short jdat[NUM_CHANNELS]) {
    mavlink_message_t message;
    mavlink_msg_rc_channels_override_pack(DB_MAVLINK_SYS_ID, 1, &message, 0, 0, jdat[0], jdat[1],
                                          jdat[2], jdat[3], jdat[4], jdat[5], jdat[6], jdat[7],
                                          jdat[8], jdat[9], jdat[10], jdat[11], jdat[12], jdat[13],
                                          0, 0, 0, 0);
    return mavlink_msg_to_send_buffer(monitor_databuffer->bytes, &message);
}

void generate_db_rc_message(uint16_t channels[NUM_CHANNELS]) {
    // Security check. Cap values. Poorly calibrated joysticks might lead to unwanted behavior!
    for (i_crc = 0; i_crc < DB_RC_NUM_CHANNELS; i_crc++) {
        if (channels[i_crc] < 1000) {
            channels[i_crc] = 1000;
        } else if (channels[i_crc] > 2000) {
            channels[i_crc] = 2000;
        }
        channels[i_crc] -= 1000;
    }

    monitor_databuffer->bytes[0] = (uint8_t) (channels[0] & 0xFF);
    monitor_databuffer->bytes[1] = (uint8_t) (((channels[0] & 0x0300) >> 8) | ((channels[1] & 0x3F) << 2));
    monitor_databuffer->bytes[2] = (uint8_t) (((channels[1] & 0x03C0) >> 6) | ((channels[2] & 0x0F) << 4));
    monitor_databuffer->bytes[3] = (uint8_t) (((channels[2] & 0x03F0) >> 4) | ((channels[3] & 0x03) << 6));
    monitor_databuffer->bytes[4] = (uint8_t) ((channels[3] & 0x03FC) >> 2);

    monitor_databuffer->bytes[5] = (uint8_t) (channels[4] & 0xFF);
    monitor_databuffer->bytes[6] = (uint8_t) (((channels[4] & 0x0300) >> 8) | ((channels[5] & 0x3F) << 2));
    monitor_databuffer->bytes[7] = (uint8_t) (((channels[5] & 0x03C0) >> 6) | ((channels[6] & 0x0F) << 4));
    monitor_databuffer->bytes[8] = (uint8_t) (((channels[6] & 0x03F0) >> 4) | ((channels[7] & 0x03) << 6));
    monitor_databuffer->bytes[9] = (uint8_t) ((channels[7] & 0x03FC) >> 2);

    monitor_databuffer->bytes[10] = (uint8_t) (channels[8] & 0xFF);
    monitor_databuffer->bytes[11] = (uint8_t) (((channels[8] & 0x0300) >> 8) | ((channels[9] & 0x3F) << 2));
    monitor_databuffer->bytes[12] = (uint8_t) (((channels[9] & 0x03C0) >> 6) | ((channels[10] & 0x0F) << 4));
    monitor_databuffer->bytes[13] = (uint8_t) (((channels[10] & 0x03F0) >> 4) | ((channels[11] & 0x03) << 6));
    monitor_databuffer->bytes[14] = (uint8_t) ((channels[11] & 0x03FC) >> 2);
    crc_rc = 0x00;
    for (i_crc = 0; i_crc < 15; i_crc++) {
        rc_crc_tbl_idx = crc_rc ^ monitor_databuffer->bytes[i_crc];
        crc_rc = crc_table_db_rc[rc_crc_tbl_idx] & 0xff;
    }
    monitor_databuffer->bytes[15] = (uint8_t) (crc_rc & 0xff);
}

void get_joy_interface_path(char *dst_joy_interface_path, int joy_interface_indx) {
    char path[] = "/dev/input/js";
    sprintf(dst_joy_interface_path, "%s%d", path, joy_interface_indx);
}

/**
 * Restore prev. calibration or calibrate using default calibration command. All based on jscal & jscal-store
 *
 * @param calibrate_comm Default calibration command eg. jscal -s xxx
 * @param joy_interface_indx Interface index of connected RC
 */
void do_calibration(char *calibrate_comm, int joy_interface_indx) {
    char cali_restore_command[CALI_COMM_SIZE + 14];
    char joy_interface_path[CALI_COMM_SIZE];
    // try to restore prev. calibration
    get_joy_interface_path(joy_interface_path, joy_interface_indx);
    sprintf(cali_restore_command, "%s %s", "jscal-restore", joy_interface_path);
    LOG_SYS_STD(LOG_INFO, "DB_CONTROL_GND: Calibrating -> %s\n", cali_restore_command);
    if (system(cali_restore_command) == 0) {
        LOG_SYS_STD(LOG_INFO, "DB_CONTROL_GND: Calibrated RC interface (restored old calibration)\n");
        return;
    } else {
        // failed to restore. Calibrate using default calibration
        if (system(calibrate_comm) == 0) {
            LOG_SYS_STD(LOG_INFO, "DB_CONTROL_GND: Calibrated RC interface with default calibration\n");
            char cali_store_comm[CALI_COMM_SIZE + 12];
            sprintf(cali_store_comm, "%s %s", "jscal-store", joy_interface_path);
            LOG_SYS_STD(LOG_INFO, "DB_CONTROL_GND: Storing calibration data -> %s\n", cali_store_comm);
            system(cali_store_comm);
        } else
            LOG_SYS_STD(LOG_WARNING, "DB_CONTROL_GND: Could not calibrate RC interface with %s\n", calibrate_comm);
    }
}

/**
 * Sets the desired RC protocol. Opens DroneBridge raw protocol sockets for transmission
 * @param new_rc_protocol 1:MSPv1, 2:MSPv2, 3:MAVLink v1, 4:MAVLink v2, 5:DB-RC
 * @param allow_rc_overwrite Set to 'Y' if you want to allow the overwrite of RC channels via a shm/external app
 * @return
 */
void conf_rc(char adapters[DB_MAX_ADAPTERS][IFNAMSIZ], int num_inf_rc, int comm_id, char db_mode, int bitrate_op,
            int frame_type, int new_rc_protocol, char allow_rc_overwrite, int adhere_80211) {
    rc_protocol = new_rc_protocol;
    en_rc_overwrite = allow_rc_overwrite == 'Y' ? true : false;
    monitor_databuffer = get_hp_raw_buffer(adhere_80211);
    for (int i = 0; i < num_inf_rc; i++) {
        raw_interfaces_rc[i] = open_db_socket(adapters[i], comm_id, db_mode, bitrate_op, DB_DIREC_DRONE,
                                              DB_PORT_CONTROLLER, frame_type);
    }
    num_interfaces = num_inf_rc;
}

/**
 * Init shared memory: RC values before sending & RC overwrite shm
 */
void open_rc_shm() {
    shm_rc_values = db_rc_values_memory_open();
    shm_rc_overwrite = db_rc_overwrite_values_memory_open();
}

/**
 * Takes the channel data (1000-2000) and builds valid packets from it. Depending on specified RC protocol.
 * Looks for channels to be overwritten by an external app and stores channel values in a shm segment
 * @param contData Values in between 1000 and 2000
 * @return
 */
int send_rc_packet(uint16_t channel_data[]) {
    if (en_rc_overwrite) {
        clock_gettime(CLOCK_MONOTONIC_COARSE, &timestamp);
        // check if shm was updated min. 100ms ago
        if (((timestamp.tv_sec - shm_rc_overwrite->timestamp.tv_sec) * (long) 1e9 + (timestamp.tv_nsec -
                                                                                     shm_rc_overwrite->timestamp.tv_nsec))
            <= 100000000L) {
            for (i_rc = 0; i_rc < NUM_CHANNELS; i_rc++) {
                if (shm_rc_overwrite->ch > 0)
                    channel_data[i_rc] = shm_rc_overwrite->ch[i_rc];
            }
        }
    }
    // Update shared memory so status module or other apps can read RC channel values
    for (i_rc = 0; i_rc < NUM_CHANNELS; i_rc++) {
        shm_rc_values->ch[i_rc] = channel_data[i_rc];
    }

    if (rc_protocol == 1) {
        generate_msp(channel_data);
        for (int i = 0; i < num_interfaces; i++) {
            db_send_hp_div(&raw_interfaces_rc[i], DB_PORT_CONTROLLER, MSP_DATA_LENTH,
                           update_seq_num(&rc_seq_number));
        }
    } else if (rc_protocol == 2) {
        generate_mspv2(channel_data);
        for (int i = 0; i < num_interfaces; i++) {
            db_send_hp_div(&raw_interfaces_rc[i], DB_PORT_CONTROLLER, MSP_V2_DATA_LENGTH,
                           update_seq_num(&rc_seq_number));
        }
    } else if (rc_protocol == 4) {
        for (int i = 0; i < num_interfaces; i++) {
            db_send_hp_div(&raw_interfaces_rc[i], DB_PORT_CONTROLLER, generate_mavlinkv2_rc_overwrite(channel_data),
                           update_seq_num(&rc_seq_number));
        }
    } else if (rc_protocol == 5) {
        generate_db_rc_message(channel_data);
        for (int i = 0; i < num_interfaces; i++) {
            db_send_hp_div(&raw_interfaces_rc[i], DB_PORT_RC, DB_RC_DATA_LENGTH, update_seq_num(&rc_seq_number));
        }
    }
    return 0;
}

void close_raw_interfaces() {
    for (int i = 0; i < DB_MAX_ADAPTERS; i++) {
        if (raw_interfaces_rc[i].db_socket != -1)
            close(raw_interfaces_rc[i].db_socket);
    }
}
