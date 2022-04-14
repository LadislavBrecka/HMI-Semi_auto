#include "irobotcreate.h"
#include "termios.h"

#include "errno.h"

int
set_interface_attribs2 (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void
set_blocking2 (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf ("error %d setting term attributes", errno);
}




iRobotCreate::iRobotCreate()
{
}
//--toto vytvara vlakno. yay
int iRobotCreate::dataProcess(void *param, ProcessDataFromCreate callback)
{

    ParamsThread *params=(ParamsThread*)malloc(sizeof(ParamsThread));
    params->classpointer=this;
    params->param=param;
    params->functionPointer=(void *)callback;
    threadID=pthread_create(&threadHandle,NULL,&robotVlakno,(void*)params);
    return 0;
}

void iRobotCreate::doSensorReadings(void *param, ProcessDataFromCreate callback)
{
    while(bezim)
    {
        CreateSensors sensor_struct;
        printf("vlakno\n");
        SentToCreate(OI_SENSORS,(unsigned char)0x06);
        usleep(100*1000);
        int k=ReceivePacketFromCreate(sensor_struct,0x06);
        if(k==0)
        {
            callback(sensor_struct,param);
        }
        else
        {
            printf("pruser %i\n",k);
        }
        usleep(80*1000);

    }
}

int iRobotCreate::ConnectToPort(char *comportT)
{
    HCom= open(comportT,O_RDWR|O_NOCTTY|O_NONBLOCK);

    if ( HCom== -1 )
    {
        printf("irobot nepripojeny\n");
        //  m_status="Chyba:  Port sa neda otvorit.";
        // potom nasleduje    Closeint(hCom);  a potom asi exit...
        return HCom;

    }
    else
    {
        set_interface_attribs2 (HCom, B57600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
        set_blocking2 (HCom, 0);                // set no blocking
      /*  struct termios settings;
        tcgetattr(HCom, &settings);

        cfsetospeed(&settings, B57600); // baud rate
        settings.c_cflag &= ~PARENB; // no parity
        settings.c_cflag &= ~CSTOPB; // 1 stop bit
        settings.c_cflag &= ~CSIZE;
        settings.c_cflag |= CS8 | CLOCAL; // 8 bits
        settings.c_lflag &= ~ICANON; // canonical mode
        settings.c_cc[VTIME]=1;
        settings.c_oflag &= ~OPOST; // raw output

        tcsetattr(HCom, TCSANOW, &settings); // apply the settings*/
        tcflush(HCom, TCOFLUSH);


    /*	DCB PortDCB;
        PortDCB.DCBlength = sizeof(DCB);  // Inicializuj položku DCBlength
        GetCommState(HCom,&PortDCB);       // Naèítaj aktuálne nastavenia
        PortDCB.BaudRate=57600;
        PortDCB.ByteSize=8;
        PortDCB.Parity=0;
        SetCommState(HCom,&PortDCB);
        PurgeComm(HCom,PURGE_TXCLEAR | PURGE_RXCLEAR);
        COMMTIMEOUTS timeouts;

        timeouts.ReadIntervalTimeout         = MAXDWORD;
        timeouts.ReadTotalTimeoutMultiplier  = 0;
        timeouts.ReadTotalTimeoutConstant    = 0;
        timeouts.WriteTotalTimeoutMultiplier = 0;
        timeouts.ReadTotalTimeoutConstant    = 0;

        SetCommTimeouts(HCom,&timeouts);*/
         printf("irobot pripojeny\n");
         usleep(100*1000);
        SentToCreate(OI_START);
         usleep(100*1000);
         SentToCreate(OI_FULL);
         usleep(100*1000);

        return HCom;
    }
}

int iRobotCreate::SentToCreate(unsigned char OI_code)
{
    DWORD Pocet;
    Pocet=write(HCom,&OI_code,1);
    printf("poslal %i\n",Pocet);
    return 0;
}

int iRobotCreate::SentToCreate(unsigned char OI_code, unsigned char data)
{
    DWORD Pocet;
    Pocet=write(HCom,&OI_code,1);

    Pocet=write(HCom,&data,1);
    return 0;
}
int iRobotCreate::SentToCreate(unsigned char OI_code, WORD data)
{
    DWORD Pocet;
    Pocet=write(HCom,&OI_code,1);
    Pocet=write(HCom,&data,2);
    return 0;
}
int iRobotCreate::SentToCreate(unsigned char OI_code, WORD data1,WORD data2)
{
    DWORD Pocet;
    Pocet=write(HCom,&OI_code,1);
    union pomoc_t
    {
        WORD data;
        unsigned char datab[2];
    }pomoc1,pomoc2;
    pomoc1.data=data1;
    pomoc2.data=data2;
    Pocet=write(HCom,&pomoc1.datab[1],1);
    Pocet=write(HCom,&pomoc1.datab[0],1);
    //Pocet=write((HCom,&data2,2);
    Pocet=write(HCom,&pomoc2.datab[1],1);
    Pocet=write(HCom,&pomoc2.datab[0],1);
    return 0;
}

int iRobotCreate::SentToCreate(unsigned char OI_code,int NumOfBytes, WORD data1,WORD data2)
{
    union pomoc_t
    {
        WORD data;
        unsigned char datab[2];
    }pomoc1,pomoc2;
DWORD Pocet;
    pomoc1.data=data1;
    pomoc2.data=data2;
    switch(NumOfBytes)
    {
        case 0: SentToCreate(HCom,OI_code);
            break;
        case 1: SentToCreate(HCom,OI_code, pomoc1.datab[1]);
            break;
        case 2:SentToCreate(HCom,OI_code,data1);
            break;
case 3:SentToCreate(HCom,OI_code,data1);
Pocet=write(HCom,&pomoc1.datab[1],1);
            break;
        case 4:SentToCreate(HCom,OI_code, data1,data2);
            break;
        default:
            break;
    }
    return 0;
}

int iRobotCreate::SentToCreate(unsigned char OI_code,unsigned char NumOfBytes, unsigned char *data)
{
    DWORD Pocet;
    Pocet=write(HCom,&OI_code,1);
    Pocet=write(HCom,&NumOfBytes,2);
    Pocet=write(HCom,&data,NumOfBytes);
    return 0;

}

int iRobotCreate::ReceivePacketFromCreate(CreateSensors &IO_SENSORS_CREATE,unsigned char packet)
{
    unsigned char *dataR;
    DWORD pocet=0;
   dataR=(unsigned char*)malloc(100);


     pocet=read(HCom,&dataR[pocet],100-pocet);


    if(pocet==OI_PacketSize[packet])
    {

    DecodeSensorsFromPacket(IO_SENSORS_CREATE,packet,dataR);
        free(dataR);
    return 0;
    }
    free(dataR);
	usleep(10*1000);
	tcflush(HCom, TCOFLUSH);
 //   printf("doslo %i\n",pocet);
    return -1;
}

int iRobotCreate::DecodeSensorsFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char packet,unsigned char *data)
{
    int i=0;
    int index=0;
    for(i=OI_PacketID[packet][0];i<=OI_PacketID[packet][1];i++)
    {
        switch(i)
        {
            case 7:
                DecodeBumsAndWheelsFromPacket(IO_SENSORS_CREATE,*(data+index));
                index++;
                break;
            case 8:
                DecodeWallFromPacket(IO_SENSORS_CREATE,*(data+index));
                index++;
                break;
            case 9:
                DecodeCliffLeftFromPacket(IO_SENSORS_CREATE,*(data+index));
                index++;
                break;
            case 10:
                DecodeCliffFrontLeftFromPacket(IO_SENSORS_CREATE,*(data+index));
                index++;
                break;
            case 11:
                DecodeCliffFrontRightFromPacket(IO_SENSORS_CREATE,*(data+index));
                index++;
                break;
            case 12:
                DecodeCliffRightFromPacket(IO_SENSORS_CREATE,*(data+index));
                index++;
                break;
            case 13:
                DecodeVirtualWallFromPacket(IO_SENSORS_CREATE,*(data+index));
                index++;
                break;
            case 14:
                DecodeLSDandWheelOvercurrentFromPacket(IO_SENSORS_CREATE,*(data+index));
                index++;
                break;
            case 15:
                index++;
                break;
            case 16:
                index++;
                break;
            case 17:
                DecodeIRFromPacket(IO_SENSORS_CREATE,*(data+index));
                index++;
                break;
            case 18:
                DecodeButtonsFromPacket(IO_SENSORS_CREATE,*(data+index));
                index++;
                break;
            case 19:
                DecodeDistanceFromPacket(IO_SENSORS_CREATE,(data+index));
                index+=2;
                break;
            case 20:DecodeAngleFromPacket(IO_SENSORS_CREATE,(data+index));
                index+=2;
                break;
            case 21:
                DecodeChargingStateFromPacket(IO_SENSORS_CREATE,*(data+index));
                index++;
                break;
            case 22:
                DecodeVoltageFromPacket(IO_SENSORS_CREATE,(data+index));
                index+=2;
                break;
            case 23:
                DecodeCurrentFromPacket(IO_SENSORS_CREATE,(data+index));
                index+=2;
                break;
            case 24:
                DecodeBatteryTemperatureFromPacket(IO_SENSORS_CREATE,*(data+index));
                index++;
                break;
            case 25:
                DecodeBatteryChargeFromPacket(IO_SENSORS_CREATE,(data+index));
                index+=2;
                break;
            case 26:
                DecodeBatteryCapacityFromPacket(IO_SENSORS_CREATE,(data+index));
                index+=2;
                break;
            case 27:
                DecodeWallSignalFromPacket(IO_SENSORS_CREATE,(data+index));
                index+=2;
                break;
            case 28:
                DecodeCliffLeftSignalFromPacket(IO_SENSORS_CREATE,(data+index));
                index+=2;
                break;
            case 29:
                DecodeCliffFrontLeftSignalFromPacket(IO_SENSORS_CREATE,(data+index));
                index+=2;
                break;
            case 30:
                DecodeCliffFrontRightSignalFromPacket(IO_SENSORS_CREATE,(data+index));
                index+=2;
                break;
            case 31:
                DecodeCliffRightSignalFromPacket(IO_SENSORS_CREATE,(data+index));
                index+=2;
                break;
            case 32:
                DecodeCargobayDigitalInputFromPacket(IO_SENSORS_CREATE,*(data+index));
                index++;
                break;
            case 33:
                DecodeCargobayAnalogSignalFromPacket(IO_SENSORS_CREATE,(data+index));
                index+=2;
                break;
            case 34:
                DecodeChargingSourceAvailableFromPacket(IO_SENSORS_CREATE,*(data+index));
                index++;
                break;
            case 35:
                DecodeOImodeFromPacket(IO_SENSORS_CREATE,*(data+index));
                index++;
                break;
            case 36:
                DecodeSongNumberFromPacket(IO_SENSORS_CREATE,*(data+index));
                index++;
                break;
            case 37:
                DecodeSongPlayingFromPacket(IO_SENSORS_CREATE,*(data+index));
                index++;
                break;
            case 38:
                DecodeNumberOfStreamPacketsFromPacket(IO_SENSORS_CREATE,*(data+index));
                index++;
                break;
            case 39:
                DecodeRequestedVelocityFromPacket(IO_SENSORS_CREATE,(data+index));
                index+=2;
                break;
            case 40:
                DecodeRequestedRadiusFromPacket(IO_SENSORS_CREATE,(data+index));
                index+=2;
                break;
            case 41:
                DecodeRequestedRightVelocityFromPacket(IO_SENSORS_CREATE,(data+index));
                index+=2;
                break;
            case 42:
                DecodeRequestedLeftVelocityFromPacket(IO_SENSORS_CREATE,(data+index));
                index+=2;
                break;
            default:
                break;
        }
    }
    return 0;
}

void iRobotCreate::DecodeBumsAndWheelsFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data) //id7
{
    IO_SENSORS_CREATE.BumpRight=data & 0x01; // bitove and, iba ak je bit z najnizsou vahou rovny 1 bude vysledok rovny 1
    IO_SENSORS_CREATE.BumpLeft=(data & 0x02)/2;
    IO_SENSORS_CREATE.WheelpdropRight=(data & 0x04)/4;
    IO_SENSORS_CREATE.WheelpdropLeft=(data & 0x08)/8;
    IO_SENSORS_CREATE.WheelpdropCaster=(data & 0x10)/16;
}

void iRobotCreate::DecodeWallFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data) //id8
{
    IO_SENSORS_CREATE.Wall=data;
}
void iRobotCreate::DecodeCliffLeftFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data) //id9
{
    IO_SENSORS_CREATE.CliffLeft=data;
}

void iRobotCreate::DecodeCliffFrontLeftFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id10
{
    IO_SENSORS_CREATE.CliffFrontLeft=data;
}

void iRobotCreate::DecodeCliffFrontRightFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id11
{
    IO_SENSORS_CREATE.CliffFrontRight=data;
}

void iRobotCreate::DecodeCliffRightFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id12
{
    IO_SENSORS_CREATE.CliffRight=data;
}

void iRobotCreate::DecodeVirtualWallFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id13
{
    IO_SENSORS_CREATE.VirtualWall=data;
}

void iRobotCreate::DecodeLSDandWheelOvercurrentFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id14
{
    IO_SENSORS_CREATE.LSD1overcurrent=data & 0x01; // bitove and, iba ak je bit z najnizsou vahou rovny 1 bude vysledok rovny 1
    IO_SENSORS_CREATE.LSD0overcurrent=(data & 0x02)/2;
    IO_SENSORS_CREATE.LSD2overcurrent=(data & 0x04)/4;
    IO_SENSORS_CREATE.RightWheelovercurrent=(data & 0x08)/8;
    IO_SENSORS_CREATE.LeftWheelovercurrent=(data & 0x10)/16;
}

void iRobotCreate::DecodeIRFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id17
{
    IO_SENSORS_CREATE.IRbyte=data;

}

void iRobotCreate::DecodeButtonsFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id18
{
    IO_SENSORS_CREATE.PlayPressed=data & 0x01; // bitove and, iba ak je bit z najnizsou vahou rovny 1 bude vysledok rovny 1
    IO_SENSORS_CREATE.AdvancePressed=(data & 0x04)/4;

}

void iRobotCreate::DecodeDistanceFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id19
{
    union pomoc_t
    {
        WORD data;
        unsigned char datab[2];
    }pomoc1;
    pomoc1.datab[1]=*data;
    pomoc1.datab[0]=*(data+1);
    IO_SENSORS_CREATE.Distance=pomoc1.data;


}

void iRobotCreate::DecodeAngleFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id20
{
    union pomoc_t
    {
        WORD data;
        unsigned char datab[2];
    }pomoc1;
    pomoc1.datab[1]=*data;
    pomoc1.datab[0]=*(data+1);
    IO_SENSORS_CREATE.Angle=pomoc1.data;


}
void iRobotCreate::DecodeChargingStateFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id21
{
    IO_SENSORS_CREATE.ChargingState=data;
}

void iRobotCreate::DecodeVoltageFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id 22
{
    union pomoc_t
    {
        WORD data;
        unsigned char datab[2];
    }pomoc1;
    pomoc1.datab[1]=*data;
    pomoc1.datab[0]=*(data+1);
    IO_SENSORS_CREATE.Voltage=pomoc1.data;


}

void iRobotCreate::DecodeCurrentFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id 23
{
    union pomoc_t
    {
        WORD data;
        unsigned char datab[2];
    }pomoc1;
    pomoc1.datab[1]=*data;
    pomoc1.datab[0]=*(data+1);
    IO_SENSORS_CREATE.Current=pomoc1.data;


}

void iRobotCreate::DecodeBatteryTemperatureFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id24
{
    IO_SENSORS_CREATE.BatteryTemperature=(char)data;

}

void iRobotCreate::DecodeBatteryChargeFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id25
{
    union pomoc_t
    {
        WORD data;
        unsigned char datab[2];
    }pomoc1;
    pomoc1.datab[1]=*data;
    pomoc1.datab[0]=*(data+1);
    IO_SENSORS_CREATE.BatteryCharge=pomoc1.data;


}
void iRobotCreate::DecodeBatteryCapacityFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id26
{
    union pomoc_t
    {
        WORD data;
        unsigned char datab[2];
    }pomoc1;
    pomoc1.datab[1]=*data;
    pomoc1.datab[0]=*(data+1);
    IO_SENSORS_CREATE.BatteryCapacity=pomoc1.data;


}

void iRobotCreate::DecodeWallSignalFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id27
{
    union pomoc_t
    {
        WORD data;
        unsigned char datab[2];
    }pomoc1;
    pomoc1.datab[1]=*data;
    pomoc1.datab[0]=*(data+1);
    IO_SENSORS_CREATE.WallSignal=pomoc1.data;


}
void iRobotCreate::DecodeCliffLeftSignalFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id28
{
    union pomoc_t
    {
        WORD data;
        unsigned char datab[2];
    }pomoc1;
    pomoc1.datab[1]=*data;
    pomoc1.datab[0]=*(data+1);
    IO_SENSORS_CREATE.CliffLeftSignal=pomoc1.data;


}

void iRobotCreate::DecodeCliffFrontLeftSignalFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id29
{
    union pomoc_t
    {
        WORD data;
        unsigned char datab[2];
    }pomoc1;
    pomoc1.datab[1]=*data;
    pomoc1.datab[0]=*(data+1);
    IO_SENSORS_CREATE.CliffFrontLeftSignal=pomoc1.data;


}

void iRobotCreate::DecodeCliffFrontRightSignalFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id30
{
    union pomoc_t
    {
        WORD data;
        unsigned char datab[2];
    }pomoc1;
    pomoc1.datab[1]=*data;
    pomoc1.datab[0]=*(data+1);
    IO_SENSORS_CREATE.CliffFrontRightSignal=pomoc1.data;


}

void iRobotCreate::DecodeCliffRightSignalFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id31
{
    union pomoc_t
    {
        WORD data;
        unsigned char datab[2];
    }pomoc1;
    pomoc1.datab[1]=*data;
    pomoc1.datab[0]=*(data+1);
    IO_SENSORS_CREATE.CliffRightSignal=pomoc1.data;


}

void iRobotCreate::DecodeCargobayDigitalInputFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id32
{
    IO_SENSORS_CREATE.CargoBayDigitalInput0=data & 0x01; // bitove and, iba ak je bit z najnizsou vahou rovny 1 bude vysledok rovny 1
    IO_SENSORS_CREATE.CargoBayDigitalInput1=(data & 0x02)/2;
    IO_SENSORS_CREATE.CargoBayDigitalInput2=(data & 0x04)/4;
    IO_SENSORS_CREATE.CargoBayDigitalInput3=(data & 0x08)/8;
    IO_SENSORS_CREATE.DeviceDetect_BaudRateChange=(data & 0x10)/16;
}

void iRobotCreate::DecodeCargobayAnalogSignalFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id33
{
    union pomoc_t
    {
        WORD data;
        unsigned char datab[2];
    }pomoc1;
    pomoc1.datab[1]=*data;
    pomoc1.datab[0]=*(data+1);
    IO_SENSORS_CREATE.CargoBayAnalogSignal=pomoc1.data;


}

void iRobotCreate::DecodeChargingSourceAvailableFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id34
{
    IO_SENSORS_CREATE.InternalCharger=data & 0x01; // bitove and, iba ak je bit z najnizsou vahou rovny 1 bude vysledok rovny 1
    IO_SENSORS_CREATE.HomaBaseCharger=(data & 0x02)/2;

}

void iRobotCreate::DecodeOImodeFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id35
{
    IO_SENSORS_CREATE.OImode=data;

}

void iRobotCreate::DecodeSongNumberFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id36
{
    IO_SENSORS_CREATE.SongNumber=data;

}
void iRobotCreate::DecodeSongPlayingFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id37
{
    IO_SENSORS_CREATE.SongPlaying=data;

}

void iRobotCreate::DecodeNumberOfStreamPacketsFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id38
{
    IO_SENSORS_CREATE.NumberOfStreamPAckets=data;

}

void iRobotCreate::DecodeRequestedVelocityFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id39
{
    union pomoc_t
    {
        WORD data;
        unsigned char datab[2];
    }pomoc1;
    pomoc1.datab[0]=*data;
    pomoc1.datab[1]=*(data+1);
    IO_SENSORS_CREATE.RequestedVelocity=pomoc1.data;


}

void iRobotCreate::DecodeRequestedRadiusFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id40
{
    union pomoc_t
    {
        WORD data;
        unsigned char datab[2];
    }pomoc1;
    pomoc1.datab[0]=*data;
    pomoc1.datab[1]=*(data+1);
    IO_SENSORS_CREATE.RequestedRadius=pomoc1.data;


}

void iRobotCreate::DecodeRequestedRightVelocityFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id41
{
    union pomoc_t
    {
        WORD data;
        unsigned char datab[2];
    }pomoc1;
    pomoc1.datab[1]=*data;
    pomoc1.datab[0]=*(data+1);
    IO_SENSORS_CREATE.RequestedRightVelocity=pomoc1.data;


}

void iRobotCreate::DecodeRequestedLeftVelocityFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id42
{
    union pomoc_t
    {
        WORD data;
        unsigned char datab[2];
    }pomoc1;
    pomoc1.datab[1]=*data;
    pomoc1.datab[0]=*(data+1);
    IO_SENSORS_CREATE.RequestedLeftVelocity=pomoc1.data;


}
