#ifndef IROBOTCREATE_H
#define IROBOTCREATE_H

#include <stdio.h>
#include <stdlib.h>
#include "unistd.h"
#include "pthread.h"
#include "iostream"
#include "fcntl.h"
#include "string.h"



typedef unsigned short WORD;
typedef unsigned int DWORD;

//prikaz na start, kym neni poslany,neda sa s create nic robit ziadne data
#define OI_START 0x80
// prikaz na zmenu baud 1 byte ako data
#define OI_BAUD 0x81
//data na baud rate
#define OI_BAUD_300 0x01
#define OI_BAUD_600 0x02
#define OI_BAUD_1200 0x03
#define OI_BAUD_2400 0x04
#define OI_BAUD_4800 0x05
#define OI_BAUD_9600 0x06
#define OI_BAUD_19200 0x07
#define OI_BAUD_28800 0x08
#define OI_BAUD_38400 0x09
#define OI_BAUD_57600 0x10
#define OI_BAUD_115200 0x11
//prikaz na rezim safe,ziadne data
#define OI_SAFE 0x83
//prikaz na rezim full,ziadne data
#define OI_FULL 0x84
//prikaz na demo, 1 byte ako data
#define OI_DEMO 0x88
// data na demo
#define OI_DEMO_ABORT 0xFF
#define OI_DEMO_COVER 0x00
#define OI_DEMO_COVER_DOCK 0x01
#define OI_DEMO_SPOT 0x02
#define OI_DEMO_MOUSE 0x03
#define OI_DEMO_DRIVE_EIGHT 0x04
#define OI_DEMO_WIMP 0x05
#define OI_DEMO_HOME 0x06
#define OI_DEMO_TAG 0x07
#define OI_DEMO_PACHELBEL 0x08
#define OI_DEMO_BANJO 0x09
// prikaz na kolesa,cesta po obluku 4 byty ako data
#define OI_DRIVE 0x89
// prve dva byty pre drive- rychlost, 2byty -500 az 500 mm/s
typedef union oi_drive_velocity_t
{
    WORD data;
    unsigned char datab[2];
}oi_drive_velocity;
// druhe dva byty pre drive- polomer, 2byty -2000 az 2000 mm/s
typedef union oi_drive_radius_t
{
    WORD data;
    unsigned char datab[2];
}oi_drive_radius;
//prikaz na drive priamo, 4 datove byty
#define OI_DRIVE_DIRECT 0x91
typedef union oi_drive_direct_right_t
{
    WORD data;
    unsigned char datab[2];
}oi_drive_direct_right;
typedef union oi_drive_direct_left_t
{
    WORD data;
    unsigned char datab[2];
}oi_drive_direct_left;
//--------------------------------------
// prikaz na ovladanie led, 3byty
#define OI_LED 0x8B
// stavy led, spajanie cez bitovy &
#define OI_ADVANCE_LED_ON 0x08
#define OI_PLAY_LED_ON 0x02
//---druhy byte, farba power ledky, 0-255 green,az red, a farby medzi tym su nejake
#define OI_POWER_LED_GREEN 0x00
#define OI_POWER_LED_RED 0xFF
//--treti byte, intenzita, 0 off, 255 max
#define OI_POWER_LED_OFF 0x00
#define OI_POWER_LED_MAX 0x255
//digital output, 1 data byte
#define OI_DIGITAL_OUTPUT 0x93
#define OI_DIGITAL_OUTPUT_0 0x01
#define OI_DIGITAL_OUTPUT_1 0x02
#define OI_DIGITAL_OUTPUT_2 0x04
//PWM Low side driver, 3data bytes
#define OI_PWM_LOW_SIDE_DRIVER 0x90
// data su 3 samostatne byty, kazdy od0 do 128.

//prikaz low side drivers, 1 data byte, stav 0/1
#define OI_LOW_SIDE_DRIVER 0x8A
#define OI_LOW_SIDE_DRIVER_0 0x01
#define OI_LOW_SIDE_DRIVER_1 0x02
#define OI_LOW_SIDE_DRIVER_2 0x04

//-send ir, 1data byte 0-255
#define OI_SEND_IR 0x97
// sensors, 1 data byte urcujuci ktore data poslat, a tento prikaz posiela spat data!!!!!!
#define OI_SENSORS 0x8E
//packets 0-42, prvych 6 su specialne sety

//prikaz query list, N+1 data bytov, prvy urcuje kolko bude packetov
#define OI_QUERY_LIST 0x95
//prikaz stream, N+1 data bytov, urcuje packety, ktore sa maju posielat pravidelne
#define OI_STREAM 0x94
//pause/reesume stream,1 data byte
#define OI_PAUSE_RESUME 0x96
//datove byty prikazy
#define OI_STREAM_PAUSE 0x00
#define OI_STREAM_RESUME 0x01
//prikaz na tvorbu scriptu, N+1 datovych bytov
#define OI_SCRIPT 0x98
// prikaz  spustenie scriptu
#define OI_SCRIPT_PLAY 0x99
// prikaz na vratenie scriptu ktory je prehravany
#define OI_SHOW_SCRIPT 0xA0

//wait commandy
// wait time, rozlisenie 15ms, 1 data byte
#define OI_WAIT_TIME 0xA1
// wait distamce, 2 data byte
#define OI_WAIT_DISTANCE 0xA2
typedef union oi_wait_distance_t
{
    WORD data;
    unsigned char datab[2];
}oi_wait_distance;

//wait angle, 2 data bytes
#define OI_WAIT_ANGLE 0xA3
typedef union oi_wait_angle_t
{
    WORD data;
    unsigned char datab[2];
}oi_wait_angle;

//wait event
#define OI_WAIT_EVENT 0xA4

//
typedef struct
{
    unsigned char WheelpdropCaster;//0-1
    unsigned char WheelpdropLeft;//0-1
    unsigned char WheelpdropRight;//0-1
    unsigned char BumpLeft;//0-1
    unsigned char BumpRight;//0-1
    unsigned char Wall;//0-1
    unsigned char CliffLeft;//0-1
    unsigned char CliffFrontLeft;//0-1
    unsigned char CliffFrontRight;//0-1
    unsigned char CliffRight;//0-1
    unsigned char VirtualWall;//0-1
    unsigned char LSD0overcurrent;//0-1
    unsigned char LSD1overcurrent;//0-1
    unsigned char LSD2overcurrent;//0-1
    unsigned char RightWheelovercurrent;//0-1
    unsigned char LeftWheelovercurrent;//0-1
    unsigned char IRbyte;//0-255
    unsigned char PlayPressed; //0-1
    unsigned char AdvancePressed;//0-1
    short	Distance;//-32768-32767 mm
    short Angle;//-32768-32767 degree
    unsigned char ChargingState;//0-5
    WORD Voltage;//0-65535 mV
    short Current;//-32768-32767
    char BatteryTemperature;//-128 -127
    WORD BatteryCharge;//0-65535 mAh
    WORD BatteryCapacity;//0-65535 mAh
    WORD WallSignal; //0-4095
    WORD CliffLeftSignal; //0-4095
    WORD CliffFrontLeftSignal;//0-4095
    WORD CliffFrontRightSignal;//0-4095
    WORD CliffRightSignal;//0-4095
    unsigned char CargoBayDigitalInput0;//0-1
    unsigned char CargoBayDigitalInput1;//0-1
    unsigned char CargoBayDigitalInput2;//0-1
    unsigned char CargoBayDigitalInput3;//0-1
    unsigned char DeviceDetect_BaudRateChange;//0-1
    WORD CargoBayAnalogSignal;//0-1023
    unsigned char InternalCharger;//0-1
    unsigned char HomaBaseCharger;//0-1
    unsigned char OImode;//0-3
    unsigned char  SongNumber;//0-15
    unsigned char  SongPlaying;//0-1
    unsigned char NumberOfStreamPAckets;//0-43
    short RequestedVelocity;//-500-500 mm/s
    short RequestedRadius;//-32768-32767 mm
    short RequestedRightVelocity;//-500-500 mm/s
    short RequestedLeftVelocity;//-500-500mm/s

}CreateSensors;

typedef struct{
    int distance;
    int angle;
    double posXstara;
    double posX;
    double posYstara;
    double posY;
    double VelR;
    double VelL;
    double VelC;
    double deltaT;
}CreatePosAndVel;

typedef struct
{
    void *classpointer;
    void *param;
    void *functionPointer;

}ParamsThread;

#define Pi 3.1415

static int OI_PacketSize[43]={26,10,6,10,14,12,52,1,1,1,1,1,1,1,1,1,1,1,1,2,2,1,2,2,1,2,2,2,2,2,2,2,1,2,1,1,1,1,1,2,2,2,2};
static int OI_PacketID[43][2]={{7,26},{7,16},{17,20},{21,26},{27,34},{35,42},{7,42},{7,7},{8,8},{9,9},{10,10},{11,11},{12,12},{13,13},{14,14},{15,15},{16,16},{17,17},{18,18},{19,19},{20,20},{21,21},{22,22},{23,23},{24,24},{25,25},{26,26},{27,27},{28,28},{29,29},{30,30},{31,31},{32,32},{33,33},{34,34},{35,35},{36,36},{37,37},{38,38},{39,39},{40,40},{41,41},{42,42}};

//definicia typu pre callback funkciu... jednak ma na vstupe udaje zo senzorov a jednak void pointer aby si mohol uzivatel odovzdat co len chce
typedef int (*ProcessDataFromCreate)(CreateSensors,void*);

class iRobotCreate
{
public:
    iRobotCreate();
    int HCom;


    int ConnectToPort(char *comportT);
    int SentToCreate(unsigned char OI_code);
    int SentToCreate(unsigned char OI_code, unsigned char data);
    int SentToCreate(unsigned char OI_code, WORD data);
    int SentToCreate(unsigned char OI_code, WORD data1,WORD data2);
    int SentToCreate(unsigned char OI_code,int NumOfBytes, WORD data1,WORD data2);
    int SentToCreate(unsigned char OI_code,unsigned char NumOfBytes, unsigned char *data);

    int move(DWORD right,DWORD left){SentToCreate(OI_DRIVE_DIRECT,right,left);}
    int dataProcess(void *param, ProcessDataFromCreate callback);
private:
    int ReceivePacketFromCreate(CreateSensors &IO_SENSORS_CREATE,unsigned char packet);
    int DecodeSensorsFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char packet,unsigned char *data);
    void DecodeBumsAndWheelsFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data);
    void DecodeWallFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data);
    void DecodeCliffLeftFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data);
    void DecodeCliffFrontLeftFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data);
    void DecodeCliffFrontRightFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data);
    void DecodeCliffRightFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data);
    void DecodeVirtualWallFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data);
    void DecodeLSDandWheelOvercurrentFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data);
    void DecodeIRFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data);
    void DecodeButtonsFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data);
    void DecodeDistanceFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data);
    void DecodeAngleFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data);
    void DecodeChargingStateFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data);
    void DecodeVoltageFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data);
    void DecodeCurrentFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data);
    void DecodeBatteryTemperatureFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data);
    void DecodeBatteryChargeFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data);
    void DecodeBatteryCapacityFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data);
    void DecodeWallSignalFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data);
    void DecodeCliffLeftSignalFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data);
    void DecodeCliffFrontLeftSignalFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data);
    void DecodeCliffFrontRightSignalFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data);
    void DecodeCliffRightSignalFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data);
    void DecodeCargobayDigitalInputFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data);
    void DecodeCargobayAnalogSignalFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data);
    void DecodeChargingSourceAvailableFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data);
    void DecodeOImodeFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data);
    void DecodeSongNumberFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data);
    void DecodeSongPlayingFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data);
    void DecodeNumberOfStreamPacketsFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data);
    void DecodeRequestedVelocityFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data);
    void DecodeRequestedRadiusFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data);
    void DecodeRequestedRightVelocityFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data);
    void DecodeRequestedLeftVelocityFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data);

    pthread_t threadHandle; // handle na vlakno
    int threadID;  // id vlakna

    static void *robotVlakno(void *param)
    {
        ParamsThread *params=(ParamsThread*)param;
        iRobotCreate *classss=(iRobotCreate*)params->classpointer;
        ProcessDataFromCreate functionCallback=(ProcessDataFromCreate)params->functionPointer;
        void *userParam=params->param;
        classss->bezim=1;
        classss->doSensorReadings(userParam,functionCallback);
        return params;
    }

    static void *robotUDPVlakno(void *param)
    {
        ParamsThread *params=(ParamsThread*)param;
        iRobotCreate *classss=(iRobotCreate*)params->classpointer;
        ProcessDataFromCreate functionCallback=(ProcessDataFromCreate)params->functionPointer;
        void *userParam=params->param;
        classss->bezim=1;
        classss->doSensorReadings(userParam,functionCallback);
        return params;
    }
    void doSensorReadings(void *param, ProcessDataFromCreate callback);
public:
    int bezim;
    void closeConnection()
    {
        bezim=0;
        usleep(500*1000);
        close(HCom);

    }

};

#endif // IROBOTCREATE_H
