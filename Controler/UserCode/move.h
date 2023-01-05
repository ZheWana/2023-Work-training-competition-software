/**
 * @file move.h
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2023/1/3
  */
#ifndef CONTROLER_MOVE_H
#define CONTROLER_MOVE_H

#include "stdbool.h"

void PositionLoopSet(int status);

void MecanumMove(float disY, float disX, float spdLimit, bool waitUntilStop);

void MecanumSpeedSet(float spdY, float spdX);

#endif //CONTROLER_MOVE_H
