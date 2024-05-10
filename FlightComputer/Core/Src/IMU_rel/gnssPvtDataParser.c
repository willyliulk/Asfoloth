/*
 * gnssPvtDataParser.c
 *
 *  Created on: Oct 24, 2023
 *      Author: liu willy
 */

#include "IMU_rel/gnssPvtDataParser.h"

uint32_t makeNum(const uint8_t pvtDataStr[], uint8_t num, uint8_t offset){
    uint32_t result=0;
    for (size_t i = 0; i < num; i++)
    {
        result = (result<<8) | pvtDataStr[i+offset];
    }
    return result;
}

GnssPvtData_t gnssPvt_parse(const uint8_t pvtDataStr[]){
    GnssPvtData_t pvtDataOut={0};
    pvtDataOut.itow         = (makeNum(pvtDataStr, 4, 0));
    pvtDataOut.year         = (makeNum(pvtDataStr, 2, 4));
    pvtDataOut.month        = (makeNum(pvtDataStr, 1, 6));
    pvtDataOut.day          = (makeNum(pvtDataStr, 1, 7));
    pvtDataOut.hour         = (makeNum(pvtDataStr, 1, 8));
    pvtDataOut.minute       = (makeNum(pvtDataStr, 1, 9));
    pvtDataOut.second       = (makeNum(pvtDataStr, 1, 10));
    pvtDataOut.valid        = (makeNum(pvtDataStr, 1, 11));
    pvtDataOut.tAcc         = (makeNum(pvtDataStr, 4, 12));
    pvtDataOut.nano         = (makeNum(pvtDataStr, 4, 16));
    pvtDataOut.fixtype      = (makeNum(pvtDataStr, 1, 20));
    pvtDataOut.flags        = (makeNum(pvtDataStr, 1, 21));
    pvtDataOut.numSV        = (makeNum(pvtDataStr, 1, 22));
    pvtDataOut.Reserved1    = 0;
    pvtDataOut.longitude    = (makeNum(pvtDataStr, 4, 24));
    pvtDataOut.latitude     = (makeNum(pvtDataStr, 4, 28));
    pvtDataOut.height       = (makeNum(pvtDataStr, 4, 32));
    pvtDataOut.hMSL         = (makeNum(pvtDataStr, 4, 36));
    pvtDataOut.hAcc         = (makeNum(pvtDataStr, 4, 40));
    pvtDataOut.vAcc         = (makeNum(pvtDataStr, 4, 44));
    pvtDataOut.velN         = (makeNum(pvtDataStr, 4, 48));
    pvtDataOut.velE         = (makeNum(pvtDataStr, 4, 52));
    pvtDataOut.velD         = (makeNum(pvtDataStr, 4, 56));
    pvtDataOut.gSpeed       = (makeNum(pvtDataStr, 4, 60));
    pvtDataOut.headMot      = (makeNum(pvtDataStr, 4, 64));
    pvtDataOut.sAcc         = (makeNum(pvtDataStr, 4, 68));
    pvtDataOut.headAcc      = (makeNum(pvtDataStr, 4, 72));
    pvtDataOut.headVeh      = (makeNum(pvtDataStr, 4, 76));
    pvtDataOut.gdop         = (makeNum(pvtDataStr, 2, 80));
    pvtDataOut.pdop         = (makeNum(pvtDataStr, 2, 82));
    pvtDataOut.tdop         = (makeNum(pvtDataStr, 2, 84));
    pvtDataOut.vdop         = (makeNum(pvtDataStr, 2, 86));
    pvtDataOut.hdop         = (makeNum(pvtDataStr, 2, 88));
    pvtDataOut.ndop         = (makeNum(pvtDataStr, 2, 90));
    pvtDataOut.edop         = (makeNum(pvtDataStr, 2, 92));

    return pvtDataOut;
}
