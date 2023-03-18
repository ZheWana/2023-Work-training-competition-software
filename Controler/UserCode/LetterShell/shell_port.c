/**
 * @file shell_port.c
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2023/1/6
  */
#include <stdlib.h>
#include "shell_port.h"
#include "string.h"
#include "stdio.h"
#include "printf.h"
#include "shell.h"
#include "move.h"

#include "utils.h"

extern CCB_Typedef CarInfo;

short uart_charPut(char *data, unsigned short len) {
    HAL_UART_Transmit(&huart5, (const uint8_t *) data, len, HAL_MAX_DELAY);
    return len;
}

uint8_t set(int argc, char *argv[]) {
    if (argc == 1) {
        printf("need an argument!\r\n");
        return 1;
    } else {
        for (int i = 1; i < argc; i++) {
            if (!strcasecmp("cpx", argv[i])) {
                CarInfo.cpPidX.kp = (float) atof(argv[i + 1]);
                printf("cpPidX.kp is set to %f\r\n", (float) atof(argv[i + 1]));
            } else if (!strcasecmp("cix", argv[i])) {
                CarInfo.cpPidX.ki = (float) atof(argv[i + 1]);
                printf("cpPidX.ki is set to %f\r\n", (float) atof(argv[i + 1]));
            } else if (!strcasecmp("cdx", argv[i])) {
                CarInfo.cpPidX.kd = (float) atof(argv[i + 1]);
                printf("cpPidX.kd is set to %f\r\n", (float) atof(argv[i + 1]));
            } else if (!strcasecmp("cax", argv[i])) {
                CarInfo.cpPidX.ctr.aim = (float) atof(argv[i + 1]);
                printf("cpPidX.aim is set to %f\r\n", (float) atof(argv[i + 1]));
            } else if (!strcasecmp("cpy", argv[i])) {
                CarInfo.cpPidY.kp = (float) atof(argv[i + 1]);
                printf("cpPidY.kp is set to %f\r\n", (float) atof(argv[i + 1]));
            } else if (!strcasecmp("ciy", argv[i])) {
                CarInfo.cpPidY.ki = (float) atof(argv[i + 1]);
                printf("cpPidY.ki is set to %f\r\n", (float) atof(argv[i + 1]));
            } else if (!strcasecmp("cdy", argv[i])) {
                CarInfo.cpPidY.kd = (float) atof(argv[i + 1]);
                printf("cpPidY.kd is set to %f\r\n", (float) atof(argv[i + 1]));
            } else if (!strcasecmp("cay", argv[i])) {
                CarInfo.cpPidY.ctr.aim = (float) atof(argv[i + 1]);
                printf("cpPidY.aim is set to %f\r\n", (float) atof(argv[i + 1]));
            } else if (!strcasecmp("ap", argv[i])) {
                CarInfo.avPid.kp = (float) atof(argv[i + 1]);
                printf("cpPidY.kp is set to %f\r\n", (float) atof(argv[i + 1]));
            } else if (!strcasecmp("ai", argv[i])) {
                CarInfo.avPid.ki = (float) atof(argv[i + 1]);
                printf("cpPidY.ki is set to %f\r\n", (float) atof(argv[i + 1]));
            } else if (!strcasecmp("ad", argv[i])) {
                CarInfo.avPid.kd = (float) atof(argv[i + 1]);
                printf("cpPidY.kd is set to %f\r\n", (float) atof(argv[i + 1]));
            } else if (!strcasecmp("aa", argv[i])) {
                CarInfo.avPid.ctr.aim = (float) atof(argv[i + 1]);
                printf("cpPidY.aim is set to %f\r\n", (float) atof(argv[i + 1]));
            }
        }
    }
    return 0;
}

uint8_t motormove(int argc, char *argv[]) {
    if (argc == 1) {
        printf("need an argument!\r\n");
        return 1;
    } else {
        float a = (float) atof(argv[1]);
        float b = (float) atof(argv[2]);
        float c = (float) atof(argv[3]);

        printf("disY = %f,disX = %f,spdLimit = %f\r\n", a, b, c);
        MecanumMove(a, b, c, 0);

    }
    return 0;
}

uint8_t mapmove(int argc, char *argv[]) {
    if (argc == 1) {
        printf("need an argument!\r\n");
        return 1;
    } else {
        float a = (float) atof(argv[1]);
        float b = (float) atof(argv[2]);
        float c = (float) atof(argv[3]);

        printf("disY = %f,disX = %f,spdLimit = %f\r\n", a, b, c);
        MapMove(a, b, c, 0);

    }
    return 0;
}

uint8_t rota(int argc, char *argv[]) {
    if (argc == 1) {
        printf("need an argument!\r\n");
        return 1;
    } else {
        float a = (float) atof(argv[1]);
        float b = (float) atof(argv[2]);

        printf("dig = %f,spdLimit = %f\r\n", a, b);
        MecanumRotate(a, b, 0);

    }
    return 0;
}


uint8_t ed(int argc, char *argv[]) {
    CarInfo.SerialOutputEnable = 0;
    return 0;
}

uint8_t op(int argc, char *argv[]) {
    CarInfo.SerialOutputEnable = 1;
    return 0;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN), motormove, motormove,
                 Move the car under the car coordinate system);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN), mapmove, mapmove,
                 Move the carunder the map coordinate system);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN), rota, rota,
                 Rotate the car);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN), ed, ed,
                 Suspend the SerialOutput Task);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN), op, op,
                 Resume the SerialOutput Task);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN), set, set,
                 Set the Variable);
