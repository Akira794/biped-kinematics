#ifndef _LINK_H_
#define _LINK_H_

#include "LinkParameter.h"

typedef struct
{
    char *joint_name;
    Vec3f p;
    Mat3f R;
    Vec3f a;
    Vec3f b;
}pLink;

void pLink_setJointInfo(pLink *const link)
{
    int32_t i, j;
    for(i = 0; i < LINK_NUM; i++)
    {
        for(j = 0; j<3; j++)
        {
            int32_t axis = AXIS[i][j];
            float32_t pos = ARMPOS[i][j];
            zVec3f_setElem(link[i].a, j, axis);
            zVec3f_setElem(link[i].b, j, pos);
        }
    }
}
