#ifndef _LINKPARAMETER_H_
#define _LINKPARAMETER_H_

#include "common/CommonDef.h"

enum
{
    BASE,
    JY1,
    JR2,
    JR3,
    JP4,
    JR5,
    JP6,
    JH,
    LINK_NUM
};

CONST int32_t AXIS[LINK_NUM][3] =
{
    { 0.0f, 0.0f, 0.0f},//BASE
    { 0.0f, 0.0f, 1.0f},//JY1
    { 1.0f, 0.0f, 0.0f},//JR2
    { 1.0f, 0.0f, 0.0f},//JR3
    { 0.0f, 1.0f, 0.0f},//JP4
    { 1.0f, 0.0f, 0.0f},//JR5
    { 0.0f, 1.0f, 0.0f},//JP6
    { 0.0f, 0.0f, 0.0f},//JH 0.1m
};

CONST float32_t ARMPOS[LINK_NUM][3] =
{
    {{0.0f, 0.0f, 0.0f},
    {{0.0f, 0.0f, 0.198f},//JY1
    {{0.0f, 0.1f, 0.122f},{ 1.0f, 0.0f, 0.0f}},//JR2
    {{0.0f, 0.0f, 0.4f},{ 1.0f, 0.0f, 0.0f}},//JR3
    {{0.0f, 0.0f, 0.0f},{ 0.0f, 1.0f, 0.0f}},//JP4
    {{0.0f, 0.4f, 0.0f},{ 1.0f, 0.0f, 0.0f}},//JR5
    {{0.0f, 0.065f, 0.0f},{ 0.0f, 1.0f, 0.0f}},//JP6
    {{0.0f, 0.1f, 0.0f},{ 0.0f, 0.0f, 0.0f}},//JH 0.1m
};
#endif /* LINKPARAMETER_H_*/
