/*
 *   This file is part of DroneBridge: https://github.com/seeul8er/DroneBridge
 *
 *   Copyright 2018 Wolfgang Christl
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

#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <zconf.h>
#include <fcntl.h>
#include <stdint.h>
#include <errno.h>
#include <memory.h>
#include <stdlib.h>
#include <linux/joystick.h>
#include "opentx.h"
#include "../common/db_protocol.h"
#include "../common/db_common.h"
#include "parameter.h"
#include "rc_ground.h"

static volatile int keep_running = 1;
static uint8_t  ctrAxes = 0;
static uint8_t  ctrBtns = 0;

struct _map {
    uint8_t ctr;       // 0x80:axis, others:button combinations
    int8_t  idx[3];
};
static struct _map joyMaps[NUM_CHANNELS];


void custom_signal_handler(int dummy) {
    keep_running = 0;
}

void loadCfg(char *cfg) {
    const char *label[] = {
        "aileron",
        "elevator",
        "throttle",
        "rudder",
        "aux1",
        "aux2",
        "aux3",
        "aux4",
        "aux5",
        "aux6",
        "aux7",
        "aux8",
        "aux9",
        "aux10"
    };

    FILE *fp = fopen(cfg, "rt");
    if (fp) {
        char line[100];
        char arg[4][30];

        memset(&joyMaps, 0x0, sizeof(joyMaps));

        while (fgets(line, sizeof(line), fp) > 0) {
            memset(arg, 0, sizeof(arg));
            sscanf(line, "%s %s %s %s", arg[0], arg[1], arg[2], arg[3]);

            for (int i = 0; i < sizeof(label) / sizeof(char *); i++) {
                if (strncmp(arg[0], label[i], strlen(label[i])) == 0) {
                    if (strncmp(arg[1], "axis", 4) == 0) {
                        joyMaps[i].ctr    = 0x80;
                        joyMaps[i].idx[0] = atoi(&arg[1][4]);
                        joyMaps[i].idx[1] = (strncmp(arg[2], "invert", 6) == 0) ? -1 : 1;   // axis invert
                    } else {
                        for (int j = 1; j < 4; j++) {
                            if (strncmp(arg[j], "button", 6) == 0) {
                                joyMaps[i].ctr++;
                                joyMaps[i].idx[j - 1] = 8 + atoi(&arg[j][6]);
                            }
                        }
                    }
                }
            }
        }
        fclose(fp);
    } else {
        // default idx
        for (int i = 0; i < sizeof(label) / sizeof(char *); i++) {
            if (i < 8) {
                joyMaps[i].ctr    = 0x80;
                joyMaps[i].idx[0] = i;
                joyMaps[i].idx[1] = 1;
            } else {
                joyMaps[i].ctr    = 1;
                joyMaps[i].idx[0] = i;
            }
        }
    }

#if 1
    for (int i = 0; i < sizeof(label) / sizeof(char *); i++) {
        printf("%s %s %2d %2d %2d\n", label[i], ((joyMaps[i].ctr == 0x80) ? "axis" : "btn"), joyMaps[i].idx[0], joyMaps[i].idx[1], joyMaps[i].idx[2]);
    }
#endif
}

/**
 * Look for the OpenTX controller on the given interface. Reinitialize if it was unplugged.
 *
 * @param joy_interface_indx Number of the joystick interface
 * @param calibrate_comm The command to be executed to calibrate the OpenTX controller
 * @return The file descriptor
 */
int initialize_opentx(int joy_interface_indx, char *cfg) {
    int fd;
    char path_interface_joystick[500];  // eg. /dev/input/js0 with 0 as the interface index
    get_joy_interface_path(path_interface_joystick, joy_interface_indx);
    LOG_SYS_STD(LOG_INFO, "DB_CONTROL_GND: Waiting for OpenTX RC to be detected on: %s\n", path_interface_joystick);
    do {
        usleep(100000);
        fd = open(path_interface_joystick, O_RDONLY | O_NONBLOCK);
    } while (fd < 0 && keep_running);
    LOG_SYS_STD(LOG_INFO, "DB_CONTROL_GND: Opened joystick interface!\n");
    char calibrate_comm[CALI_COMM_SIZE];
    strcpy(calibrate_comm, DEFAULT_OPENTX_CALIBRATION);
    do_calibration(calibrate_comm, joy_interface_indx);

    if (ioctl(fd, JSIOCGAXES, &ctrAxes) == -1) {
        ctrAxes = 4;
    }
    if (ioctl(fd, JSIOCGBUTTONS, &ctrBtns) == -1) {
        ctrBtns = 4;
    }
    LOG_SYS_STD(LOG_INFO, "DB_CONTROL_GND: axis:%d, buttons:%d\n", ctrAxes, ctrBtns);
    loadCfg(cfg);

    return fd;
}

/**
 * Transform the values read from the RC to values between 1000 and 2000
 *
 * @param value The value read from the interface
 * @param adjustingValue A extra value that might add extra exponential behavior to the sticks etc.
 * @return 1000<=return_value<=2000
 */
uint16_t normalize_opentx(int16_t value) {
    return (uint16_t) (((500 * value) / MAX) + 1500);
}

void mapData(int16_t *pInJoy, uint16_t *pOutTx) {
    for (int i = 0; i < NUM_CHANNELS; i++) {
        uint16_t div = (1 << joyMaps[i].ctr) - 1;

        if (joyMaps[i].ctr == 0x80) {   // axis
            pOutTx[i] = normalize_opentx(pInJoy[joyMaps[i].idx[0]] * joyMaps[i].idx[1]);
        } else {                        // button
            uint16_t val = 0;

            for (int j = 0; j < joyMaps[i].ctr; j++) {
                val |= (pInJoy[joyMaps[i].idx[j]] << j);
            }
            pOutTx[i] = 1000 + (1000 * val / div);
        }
    }
}

/**
 * Read and send RC commands using a OpenTX based radio
 *
 * @param Joy_IF Joystick interface as specified by jscal interface index of the OpenTX based radio connected via USB
 * @param frequency_sleep Time to sleep between every RC value read & send
 */
void opentx(int Joy_IF, struct timespec frequency_sleep, char *cfg) {
    signal(SIGINT, custom_signal_handler);
    struct js_event e;
    uint16_t txData[NUM_CHANNELS];
    struct timespec tim_remain;
    int16_t joystickData[40] = {0};

    int fd = initialize_opentx(Joy_IF, cfg);
    LOG_SYS_STD(LOG_INFO, "DB_CONTROL_GND: DroneBridge OpenTX - starting!\n");
    while (keep_running) //send loop
    {
        nanosleep(&frequency_sleep, &tim_remain);
        while (read(fd, &e, sizeof(e)) > 0)   // go through all events occurred
        {
            e.type &= ~JS_EVENT_INIT; /* ignore synthetic events */
            if (e.type == JS_EVENT_AXIS) {
                joystickData[e.number] = e.value;
            } else if (e.type == JS_EVENT_BUTTON) {
                joystickData[8 + e.number] = e.value;
            }
        }

        int myerror = errno;
        if (myerror != EAGAIN) {
            if (myerror == ENODEV) {
                LOG_SYS_STD(LOG_WARNING, "DB_CONTROL_GND: Joystick was unplugged! Retrying...\n");
                fd = initialize_opentx(Joy_IF, cfg);
            } else {
                LOG_SYS_STD(LOG_ERR, "DB_CONTROL_GND: Error: %s\n", strerror(myerror));
            }
        }
        mapData(joystickData, txData);
        send_rc_packet(txData);
    }
    close(fd);
    close_raw_interfaces();
}