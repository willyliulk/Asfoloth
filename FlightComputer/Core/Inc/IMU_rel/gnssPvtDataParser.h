/*
 * gnssPvtDataParser.h
 *
 *  Created on: Oct 13, 2023
 *      Author: liu willy
 */

#ifndef GNSS_PVT_DATA_PARSER_H
#define GNSS_PVT_DATA_PARSER_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

enum Fixtype{
    NoFix,
    Dead_Reckoning, //Dead Reckoning only
    Fix2D,
    Fix3D,
    Gnss_Dead_Reckoning, //GNSS + dead reckoning combined
    Time_only_fix, //Time only fix
};//other is reserved
typedef enum Fixtype Fixtype_t;

struct GnssPvtData
{
    uint32_t    itow;       //GPS time of week
    uint16_t    year;       //Year (UTC)
    uint8_t     month;      //Month (UTC)
    uint8_t     day;        //Day of the month (UTC)
    uint8_t     hour;       //Hour of the day 0..23 (UTC)
    uint8_t     minute;     //Minute of hour 0..59 (UTC)
    uint8_t     second;     //Seconds of minute 0..60 (UTC)
    uint8_t     valid;      //Validity flags
    uint32_t    tAcc;       //Time accuracy estimate (UTC)
    int32_t     nano;       //Fraction of second -1e-9 .. 1e-9
    Fixtype_t   fixtype;    //GNSS fix type
    uint8_t     flags;      //Fix Status Flags
    uint8_t     numSV;      //Number of satellites used in the navigation solution
    uint8_t     Reserved1;
    int32_t     longitude;  //Longitude
    int32_t     latitude;   //Latitude
    int32_t     height;     //Height above ellipsoid
    int32_t     hMSL;       //Height above mean sea level
    uint32_t    hAcc;       //Horizontal accuracy estimate
    uint32_t    vAcc;       //Vertical accuracy estimate
    int32_t     velN;       //NED north velocity
    int32_t     velE;       //NED east velocity
    int32_t     velD;       //NED down velocity
    int32_t     gSpeed;     //2D ground speed
    int32_t     headMot;    //2D heading of motion
    uint32_t    sAcc;       //Speed accuracy estimate
    uint32_t    headAcc;    //Heading accuracy estimate (both motion and vehicle)
    int32_t     headVeh;    //2D heading of vehicle
    uint16_t    gdop;       //Geometric DOP
    uint16_t    pdop;       //Position DOP
    uint16_t    tdop;       //Time DOP
    uint16_t    vdop;       //Vertical DOP
    uint16_t    hdop;       //Horizontal DOP
    uint16_t    ndop;       //Northing DOP
    uint16_t    edop;       //Easting DOP
};
//Validity flags
//|0:UTC Date is valid|1:UTC Time of Day is valid|2:UTC Time of Day has been fully resolved (i.e. no seconds uncertainty)|
//Fix Status Flags
//|0:valid fix|1:differential corrections are applied|2~4:reserved|5:heading of vehicle is valid|6~7:carrier phase range solution status|
//carrier phase range solution status:
//0:no solution   1:solution with floating ambiguities  2:solution with fixed ambiguities

typedef struct GnssPvtData GnssPvtData_t;

uint32_t makeNum(const uint8_t pvtDataStr[], uint8_t num, uint8_t offset);
GnssPvtData_t gnssPvt_parse(const uint8_t pvtDataStr[]);


#endif
