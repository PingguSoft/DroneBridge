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

#define NUM_BUTTON_COMB     4
#define TYPE_AXIS_NORMAL    1
#define TYPE_AXIS_INVERT    2
#define TYPE_BUTTON         3
#define TYPE_BUTTON_TOGGLE  4
#define TYPE_SWITCH         5

static volatile int keep_running = 1;
static uint8_t  ctrAxes = 0;
static uint8_t  ctrBtns = 0;

struct joymap {
    uint8_t  ctr;
    uint8_t  type[NUM_BUTTON_COMB];
    int8_t   idx[NUM_BUTTON_COMB];
    uint16_t lastState[NUM_BUTTON_COMB];
    uint16_t lastBtn[NUM_BUTTON_COMB];
};

static struct joymap _joyMaps[NUM_CHANNELS];


void custom_signal_handler(int dummy) {
    keep_running = 0;
}

char *nextParam(char *p) {
    // find space
    while (*p != '\0' && *p != ' ' && *p != '\t') {
        p++;
    }
    // find non space
    while (*p != '\0' && (*p == ' ' || *p == '\t')) {
        p++;
    }
    return p;
}

void loadCfg(char *cfg) {
    const char *label[NUM_CHANNELS] = {
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

    memset(_joyMaps, 0, sizeof(_joyMaps));
    FILE *fp = fopen(cfg, "rt");
    if (fp) {
        char line[100];
        char *ptr;

        while (fgets(line, sizeof(line), fp) > 0) {
            ptr = line;
            for (int i = 0; i < NUM_CHANNELS; i++) {
                if (strncmp(ptr, label[i], strlen(label[i])) == 0) {
                    ptr = nextParam(ptr);
                    if (strncmp(ptr, "axis", 4) == 0) {
                        _joyMaps[i].ctr     = 1;
                        _joyMaps[i].type[0] = (ptr[4] == '-') ? TYPE_AXIS_INVERT : TYPE_AXIS_NORMAL;
                        _joyMaps[i].idx[0]  = atoi(&ptr[5]);
                    } else {
                        for (int j = 0; j < NUM_BUTTON_COMB; j++) {
                            if (strncmp(ptr, "button", 6) == 0) {
                                _joyMaps[i].ctr++;
                                if (ptr[6] == '^') {
                                    _joyMaps[i].type[j] = TYPE_BUTTON_TOGGLE;
                                    _joyMaps[i].idx[j]  = 8 + atoi(&ptr[7]);
                                } else {
                                    _joyMaps[i].type[j] = TYPE_BUTTON;
                                    _joyMaps[i].idx[j]  = 8 + atoi(&ptr[6]);
                                }
                            } else if (strncmp(ptr, "switch", 6) == 0) {
                                _joyMaps[i].ctr++;
                                _joyMaps[i].type[j] = TYPE_SWITCH;
                                _joyMaps[i].idx[j]  = 8 + atoi(&ptr[6]);
                            }
                            ptr = nextParam(ptr);
                        }
                    }
                }
            }
        }
        fclose(fp);
    } else {
        // default mapping
        for (int i = 0; i < NUM_CHANNELS; i++) {
            if (i < 4) {
                _joyMaps[i].ctr     = 1;
                _joyMaps[i].type[0] = TYPE_AXIS_NORMAL;
                _joyMaps[i].idx[0]  = i;
            } else {
                _joyMaps[i].ctr     = 1;
                _joyMaps[i].type[0] = TYPE_BUTTON;
                _joyMaps[i].idx[0]  = 8 + i;
            }
        }
    }
#if 1
    for (int i = 0; i < sizeof(label) / sizeof(char *); i++) {
        if (_joyMaps[i].ctr == 0)
            continue;
            
        printf("%-8s ", label[i]);
        
        char *type;
        for (int j = 0; j < _joyMaps[i].ctr; j++) {
            switch (_joyMaps[i].type[j]) {
                case TYPE_AXIS_NORMAL:
                    type = "axis +";
                    break;
                    
                case TYPE_AXIS_INVERT:
                    type = "axis -";
                    break;
                    
                case TYPE_BUTTON:
                    type = "button";
                    break;
                    
                case TYPE_BUTTON_TOGGLE:
                    type = "button^";
                    break;
                    
                case TYPE_SWITCH:
                    type = "switch";
                    break;
            }
            printf("%8s(%2d) ", type, _joyMaps[i].idx[j]);
        }
        printf("\n");
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
        if (_joyMaps[i].ctr == 0) {
            pOutTx[i] = 1000;
        } else if (_joyMaps[i].type[0] < TYPE_BUTTON) {
            int val = pInJoy[_joyMaps[i].idx[0]];

            if (_joyMaps[i].type[0] == TYPE_AXIS_INVERT) {
                val = -val;
            }
            pOutTx[i] = normalize_opentx(val);
            if (i < 4) {
                LOG_SYS_STD(LOG_INFO, "%d:%4d, ", i, pOutTx[i]);
            } else {
                LOG_SYS_STD(LOG_INFO, "A%d:%4d, ", i - 3, pOutTx[i]);
            }
        } else {
            uint16_t div   = (1 << _joyMaps[i].ctr) - 1;
            uint16_t round = div - 1;
            uint16_t btnVal;
            uint16_t newVal = 0;            
            uint16_t toggled;

            for (int j = 0; j < _joyMaps[i].ctr; j++) {
                btnVal  = pInJoy[_joyMaps[i].idx[j]];
                toggled = _joyMaps[i].lastState[j] ^ btnVal;
                _joyMaps[i].lastState[j] = btnVal;

                if (_joyMaps[i].type[j] == TYPE_BUTTON_TOGGLE) {
                     if (btnVal > 0 && toggled > 0) {
                        _joyMaps[i].lastBtn[j] = !_joyMaps[i].lastBtn[j];
                     }
                     btnVal = _joyMaps[i].lastBtn[j];
                }
                newVal |= (btnVal << j);
            }
            
            pOutTx[i] = 1000 + (1000 * newVal + round) / div;
            LOG_SYS_STD(LOG_INFO, "A%d:%4d, ", i - 3, pOutTx[i]);
        }
    }
    LOG_SYS_STD(LOG_INFO, "\n");
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
