#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QTimer"
#include "QPainter"
#include "math.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"
#include <QCoreApplication>
#include <QtConcurrent/QtConcurrent>

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_W)
    {
        RobotSetTranslationSpeed(350.0f);
    }

    if(event->key() == Qt::Key_A)
    {
        RobotSetRotationSpeed(PI/4.0f);
    }

    if(event->key() == Qt::Key_S)
    {
        RobotSetTranslationSpeed(0.0f);
    }

    if(event->key() == Qt::Key_D)
    {
        RobotSetRotationSpeed(-PI/4.0f);
    }

    if(event->key() == Qt::Key_X)
    {
        RobotSetTranslationSpeed(-350.0f);
    }
}

//funkcia local robot je na priame riadenie robota, ktory je vo vasej blizskoti, viete si z dat ratat polohu atd (zapnutie dopravneho oneskorenia sposobi opozdenie dat oproti aktualnemu stavu robota)
void MainWindow::localrobot(TKobukiData &sens)
{ 
    // inicializacia pri prvom spusteni
    if (initParam)
    {
        // inicializacia mojich premennych
        x = 0.0f;
        y = 0.0f;

        enc_l_prev = sens.EncoderLeft;
        enc_r_prev = sens.EncoderRight;
        f_k_prev = 0.0f;
        x_prev = 0.0f;
        y_prev = 0.0f;
        total_l = 0.0f;
        total_r = 0.0f;

        initParam = false;
    }

    // pridavok enkoderov oboch kolies
    int enc_r_diff = sens.EncoderRight - enc_r_prev;
    int enc_l_diff = sens.EncoderLeft - enc_l_prev;

    // pretecenie praveho enkodera
    if (enc_r_diff < -60000)
        enc_r_diff = 65535 + enc_r_diff;
    else if (enc_r_diff > 60000)
        enc_r_diff = enc_r_diff - 65535;

    // pretecenie laveho enkodera
    if (enc_l_diff < -60000)
        enc_l_diff = 65535 + enc_l_diff;
    else if (enc_l_diff > 60000)
        enc_l_diff = enc_l_diff - 65535;

    // pridavok vzdialenosti oboch kolies
    l_r = tickToMeter * (enc_r_diff);
    l_l = tickToMeter * (enc_l_diff);

    // vzdialenost l_k a uhol f_k
    l_k = (l_r + l_l) / 2;
    d_alfa = (l_r - l_l) / b;
    f_k = f_k_prev + d_alfa;

    // pretecenie uhla
    if (f_k > 2*PI)
        f_k -= 2*PI;
    else if (f_k < 0.0f)
        f_k += 2*PI;

    // suradnice x a y
    x = x_prev + l_k * cos(f_k);
    y = y_prev + l_k * sin(f_k);

    // ulozenie aktualnych do predoslych
    enc_l_prev = sens.EncoderLeft;
    enc_r_prev = sens.EncoderRight;
    f_k_prev = f_k;
    x_prev = x;
    y_prev = y;
}

// funkcia local laser je naspracovanie dat z lasera(zapnutie dopravneho oneskorenia sposobi opozdenie dat oproti aktualnemu stavu robota)
int MainWindow::locallaser(LaserMeasurement &laserData)
{

    paintThisLidar(laserData);

    ///PASTE YOUR CODE HERE
    /// ****************
    /// mozne hodnoty v return
    /// ROBOT_VPRED
    /// ROBOT_VZAD
    /// ROBOT_VLAVO
    /// ROBOT_VPRAVO
    /// ROBOT_STOP
    /// ROBOT_ARC
    ///
    /// ****************

    return -1;
}


//--autonomousrobot simuluje slucku robota, ktora bezi priamo na robote
// predstavte si to tak,ze ste naprogramovali napriklad polohovy regulator, uploadli ste ho do robota a tam sa to vykonava
// dopravne oneskorenie nema vplyv na data
void MainWindow::autonomousrobot(TKobukiData &sens)
{

}
//--autonomouslaser simuluje spracovanie dat z robota, ktora bezi priamo na robote
// predstavte si to tak,ze ste naprogramovali napriklad sposob obchadzania prekazky, uploadli ste ho do robota a tam sa to vykonava
// dopravne oneskorenie nema vplyv na data
int MainWindow::autonomouslaser(LaserMeasurement &laserData)
{
    ///PASTE YOUR CODE HERE
    /// ****************
    ///
    ///
    /// ****************
    return -1;
}

///kamera nema svoju vlastnu funkciu ktora sa vola, ak chcete niekde pouzit obrazok, aktualny je v premennej
/// robotPicture alebo ekvivalent AutonomousrobotPicture
/// pozor na synchronizaciu, odporucam akonahle chcete robit nieco s obrazkom urobit si jeho lokalnu kopiu
/// cv::Mat frameBuf; robotPicture.copyTo(frameBuf);


//sposob kreslenia na obrazovku, tento event sa spusti vzdy ked sa bud zavola funkcia update() alebo operacny system si vyziada prekreslenie okna

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setBrush(Qt::black);
    QPen pero;
    pero.setStyle(Qt::SolidLine);
    pero.setWidth(3);
    pero.setColor(Qt::green);

    QRect mainRect     = ui->mainFrame->geometry();
    QRect lidarRect    = ui->lidarFrame->geometry();
    QRect skeletonRect = ui->skeletonFrame->geometry();

    painter.drawRect(mainRect);
    painter.setBrush(Qt::white);
    painter.drawRect(lidarRect);
    painter.setBrush(Qt::white);
    painter.drawRect(skeletonRect);
    painter.setBrush(Qt::black);

    QPoint lidarTopLeft        = lidarRect.topLeft();
    QPoint lidarBottomRight    = lidarRect.bottomRight();
    QPoint skeletonTopLeft     = skeletonRect.topLeft();
    QPoint skeletonBottomRight = skeletonRect.bottomRight();

    int lidarWidth  = lidarBottomRight.x() - lidarTopLeft.x();
    int lidarHeight = lidarBottomRight.y() - lidarTopLeft.y();

    int skeletonWidth  = skeletonBottomRight.x() - skeletonTopLeft.x();
    int skeletonHeight = skeletonBottomRight.y() - skeletonTopLeft.y();

    // draw lidar integration
    painter.setPen(pero);
    float q_dist[8] = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0};

    bool draw_robot = true;
    bool wallNear = false;
    float warn_d = 0.35;

    for(int k=0;k<paintLaserData.numberOfScans;k++)
    {
        // computing x and y of lidar points in camera space
        float f = 628.036;
        float D = paintLaserData.Data[k].scanDistance / 1000.0;
        float a = 360.0 - paintLaserData.Data[k].scanAngle;
        if (a > 360)    a -= 360;
        if (a < 0)      a += 360;

        float dist_color = ((D - 0.20) * 200.0 / (2.5 - 0.20)) + 55.0;

        if ((a >=0 && a < 27) || (a > 333 && a < 360))
        {
            if (D < 2.5f)
            {
                float Y = 0.06;
                float Z = D*cos(a * 3.14159 / 180.0);
                float X = D*sin(a * 3.14159 / 180.0);

                int x_obr = (robotPicture.cols  / 2 - (f * X) / Z);
                int y_obr = (robotPicture.rows / 2 + (f * Y) / Z);

                if (x_obr >= 640 || y_obr >= 480 || x_obr < 0 || y_obr < 0) continue;

                // change color according to distance
                if (!robotPicture.empty())
                    cv::floodFill(robotPicture, cv::Point(x_obr, y_obr), cv::Scalar(dist_color, dist_color, 255.0), 0, cv::Scalar(15, 15, 15), cv::Scalar(25, 25, 25));

                if (D < warn_d && !wallNear)
                {
                    cv::putText(robotPicture, //target image
                            "WALL", //text
                            cv::Point(x_obr, robotPicture.rows / 2), //top-left position
                            cv::FONT_HERSHEY_DUPLEX,
                            1.0,
                            CV_RGB(255, 0, 0), //font color
                            2);
                    wallNear = true;
                }
            }
        }

        if (D < warn_d && D > 0.1 && !wallNear)
        {
            if (a <= 120)
            {
                cv::putText(robotPicture, //target image
                        "!!", //text
                        cv::Point(15, robotPicture.rows / 2), //top-left position
                        cv::FONT_HERSHEY_DUPLEX,
                        1.0,
                        CV_RGB(255, 0, 0), //font color
                        2);
                wallNear = true;
            }
            if (a >120 && a<210)
            {
                cv::putText(robotPicture, //target image
                        "!!", //text
                        cv::Point(robotPicture.cols/2.0f, robotPicture.rows - 25), //top-left position
                        cv::FONT_HERSHEY_DUPLEX,
                        1.0,
                        CV_RGB(255, 0, 0), //font color
                        2);
                wallNear = true;
            }

            if (a >= 210)
            {
                cv::putText(robotPicture, //target image
                        "!!", //text
                        cv::Point(robotPicture.cols-50, robotPicture.rows / 2), //top-left position
                        cv::FONT_HERSHEY_DUPLEX,
                        1.0,
                        CV_RGB(255, 0, 0), //font color
                        2);
                wallNear = true;
            }
        }

        // classifying all quadrants occupied by barrier
        if (D < 0.8 && D > 0.15)
        {
           int q = int(a)/45 % 8 +1;
           if (q_dist[q - 1] > D)
                q_dist[q - 1] = D;    // q is from 1-8, array indexing from 0-7
        }

        // computing x and y for standalone lidar frame
        int zooming = (int)(7000 / lidarWidth);
        int dist = paintLaserData.Data[k].scanDistance / zooming;
        int xp   = lidarBottomRight.x() - (lidarWidth  / 2 + dist * sin((360.0 - paintLaserData.Data[k].scanAngle) * 3.14159 / 180.0));
        int yp   = lidarBottomRight.y() - (lidarHeight / 2 + dist * cos((360.0 - paintLaserData.Data[k].scanAngle) * 3.14159 / 180.0));

        if (draw_robot)
        {
            int xp   = lidarBottomRight.x() - (lidarWidth  / 2 + 0.0 * sin((360.0 - 0.0) * 3.14159 / 180.0));
            int yp   = lidarBottomRight.y() - (lidarHeight / 2 + 0.0 * cos((360.0 - 0.0) * 3.14159 / 180.0));

            float xp_2 = xp + 0.0f;
            float yp_2 = yp - 15.0f;

            painter.setPen(QPen(Qt::blue));
            painter.drawLine(QLine(QPoint(xp, yp), QPoint(xp_2, yp_2)));
            painter.drawEllipse(QPoint(xp, yp), 6, 6);
            draw_robot = false;
        }

        // painting lidar points to standalone frame
        if(xp < lidarBottomRight.x() && xp > lidarTopLeft.x() && yp < lidarBottomRight.y() && yp > lidarTopLeft.y())
        {
            // change color according to distance
            painter.setPen(QColor(175, 175, 175));
            painter.drawEllipse(QPoint(xp, yp), 2, 2);
        }
    }

    switch(direction)
    {
        case 1:
            cv::putText(robotPicture, //target image
                    "FORWARD", //text
                    cv::Point(robotPicture.cols/2.0f-20.0, 40.0f), //top-left position
                    cv::FONT_HERSHEY_DUPLEX,
                    0.6,
                    CV_RGB(220, 220, 220), //font color
                    2);
            break;

        case 2:
            cv::putText(robotPicture, //target image
                    "BACKWARD", //text
                    cv::Point(robotPicture.cols/2.0f-20.0f, 40.0f), //top-left position
                    cv::FONT_HERSHEY_DUPLEX,
                    0.6,
                    CV_RGB(220, 220, 220), //font color
                    2);
            break;

        case 3:
            cv::putText(robotPicture, //target image
                    "LEFT", //text
                    cv::Point(15.0f, 40.0f), //top-left position
                    cv::FONT_HERSHEY_DUPLEX,
                    0.6,
                    CV_RGB(220, 220, 220), //font color
                    2);
            break;

        case 4:
            cv::putText(robotPicture, //target image
                    "RIGHT", //text
                    cv::Point(robotPicture.cols-70.0f, 40.0f), //top-left position
                    cv::FONT_HERSHEY_DUPLEX,
                    0.6,
                    CV_RGB(220, 220, 220), //font color
                    2);
            break;
    }

    // draw main camera
    QImage imgIn = QImage((uchar*) robotPicture.data, robotPicture.cols, robotPicture.rows, robotPicture.step, QImage::Format_BGR888);
    painter.drawImage(mainRect ,imgIn,imgIn.rect());

    // skeleton drawing
    painter.setPen(QColor(175,175,175));
    for(int i = 0; i < 75; i++)
    {
        int xp = skeletonBottomRight.x() - skeletonWidth * kostricka.joints[i].x;
        int yp = skeletonTopLeft.y() + skeletonHeight * kostricka.joints[i].y;

        if(xp < skeletonBottomRight.x() && xp > skeletonTopLeft.x() && yp < skeletonBottomRight.y() && yp > skeletonTopLeft.y())
            painter.drawEllipse(QPoint(xp, yp), 2, 2);

    }
}

void MainWindow::RobotSetTranslationSpeed(float speed)
{
    if (speed > 0.0)
       direction = 1;
    else if (speed < 0.0)
        direction = 2;
    else
        direction = 0;

    std::vector<unsigned char> mess=robot.setTranslationSpeed(speed);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::RobotSetRotationSpeed(float speed)
{
    if (speed > 0.0)
       direction = 3;
    else if (speed < 0.0)
        direction = 4;
    else
        direction = 0;

    std::vector<unsigned char> mess=robot.setRotationSpeed(speed);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}



///konstruktor aplikacie, nic co tu je nevymazavajte, ak potrebujete, mozete si tu pridat nejake inicializacne parametre
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    showCamera=false;
    showLidar=true;
    showSkeleton=false;
    applyDelay=true;
    dl=0;
    stopall=1;
    updateCameraPicture=0;
    ipaddress="127.0.0.1";
    std::function<void(void)> f =std::bind(&robotUDPVlakno, (void *)this);
    robotthreadHandle=std::move(std::thread(f));
    std::function<void(void)> f2 =std::bind(&laserUDPVlakno, (void *)this);
    laserthreadHandle=std::move(std::thread(f2));

    std::function<void(void)> f3 =std::bind(&skeletonUDPVlakno, (void *)this);
    skeletonthreadHandle=std::move(std::thread(f3));

    //--ak by ste nahodou chceli konzolu do ktorej mozete vypisovat cez std::cout, odkomentujte nasledujuce dva riadky
   // AllocConsole();
   // freopen("CONOUT$", "w", stdout);


    QFuture<void> future = QtConcurrent::run([=]() {
        imageViewer();
        // Code in this block will run in another thread
    });



        Imager.start();

}
//--cokolvek za tymto vas teoreticky nemusi zaujimat, su tam len nejake skarede kody































































MainWindow::~MainWindow()
{
    stopall=0;
    laserthreadHandle.join();
    robotthreadHandle.join();
    skeletonthreadHandle.join();
    delete ui;
}









void MainWindow::robotprocess()
{

#ifdef _WIN32
    WSADATA wsaData = {0};
    int iResult = 0;
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
#else
#endif
    rob_slen = sizeof(las_si_other);
    if ((rob_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    char rob_broadcastene=1;
    DWORD timeout=100;

    setsockopt(rob_s, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof timeout);
    setsockopt(rob_s,SOL_SOCKET,SO_BROADCAST,&rob_broadcastene,sizeof(rob_broadcastene));
    // zero out the structure
    memset((char *) &rob_si_me, 0, sizeof(rob_si_me));

    rob_si_me.sin_family = AF_INET;
    rob_si_me.sin_port = htons(53000);
    rob_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    rob_si_posli.sin_family = AF_INET;
    rob_si_posli.sin_port = htons(5300);
    rob_si_posli.sin_addr.s_addr =inet_addr(ipaddress.data());//inet_addr("10.0.0.1");// htonl(INADDR_BROADCAST);
    rob_slen = sizeof(rob_si_me);
    bind(rob_s , (struct sockaddr*)&rob_si_me, sizeof(rob_si_me) );

    std::vector<unsigned char> mess=robot.setDefaultPID();
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
#ifdef _WIN32
    Sleep(100);
#else
    usleep(100*1000);
#endif
    mess=robot.setSound(440,1000);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
    unsigned char buff[50000];
    while(stopall==1)
    {

        memset(buff,0,50000*sizeof(char));
        if ((rob_recv_len = recvfrom(rob_s, (char*)&buff, sizeof(char)*50000, 0, (struct sockaddr *) &rob_si_other, &rob_slen)) == -1)
        {

            continue;
        }
        //tu mame data..zavolame si funkciu

        //     memcpy(&sens,buff,sizeof(sens));
        struct timespec t;
        //      clock_gettime(CLOCK_REALTIME,&t);

        int returnval=robot.fillData(sens,(unsigned char*)buff);
        if(returnval==0)
        {
            //     memcpy(&sens,buff,sizeof(sens));

            std::chrono::steady_clock::time_point timestampf=std::chrono::steady_clock::now();

            autonomousrobot(sens);

            if(applyDelay==true)
            {
                struct timespec t;
                RobotData newcommand;
                newcommand.sens=sens;
                //    memcpy(&newcommand.sens,&sens,sizeof(TKobukiData));
                //        clock_gettime(CLOCK_REALTIME,&t);
                newcommand.timestamp=std::chrono::steady_clock::now();;//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
                auto timestamp=std::chrono::steady_clock::now();;//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
                sensorQuerry.push_back(newcommand);
                for(int i=0;i<sensorQuerry.size();i++)
                {
                    if(( std::chrono::duration_cast<std::chrono::nanoseconds>(timestampf-sensorQuerry[i].timestamp)).count()>(2.5*1000000000))
                    {
                        localrobot(sensorQuerry[i].sens);
                        sensorQuerry.erase(sensorQuerry.begin()+i);
                        i--;
                        break;

                    }
                }

            }
            else
            {
                sensorQuerry.clear();
                localrobot(sens);
            }
        }


    }

    std::cout<<"koniec thread2"<<std::endl;
}
/// vravel som ze vas to nemusi zaujimat. tu nic nieje
/// nosy litlle bastard
void MainWindow::laserprocess()
{
#ifdef _WIN32
    WSADATA wsaData = {0};
    int iResult = 0;
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
#else
#endif
    las_slen = sizeof(las_si_other);
    if ((las_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    char las_broadcastene=1;
#ifdef _WIN32
    DWORD timeout=100;

    setsockopt(las_s, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof timeout);
    setsockopt(las_s,SOL_SOCKET,SO_BROADCAST,&las_broadcastene,sizeof(las_broadcastene));
#else
    setsockopt(las_s,SOL_SOCKET,SO_BROADCAST,&las_broadcastene,sizeof(las_broadcastene));
#endif
    // zero out the structure
    memset((char *) &las_si_me, 0, sizeof(las_si_me));

    las_si_me.sin_family = AF_INET;
    las_si_me.sin_port = htons(52999);
    las_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    las_si_posli.sin_family = AF_INET;
    las_si_posli.sin_port = htons(5299);
    las_si_posli.sin_addr.s_addr = inet_addr(ipaddress.data());;//htonl(INADDR_BROADCAST);
    bind(las_s , (struct sockaddr*)&las_si_me, sizeof(las_si_me) );
    char command=0x00;
    if (sendto(las_s, &command, sizeof(command), 0, (struct sockaddr*) &las_si_posli, rob_slen) == -1)
    {

    }
    LaserMeasurement measure;
    while(stopall==1)
    {

        if ((las_recv_len = recvfrom(las_s, (char *)&measure.Data, sizeof(LaserData)*1000, 0, (struct sockaddr *) &las_si_other, &las_slen)) == -1)
        {

            continue;
        }
        measure.numberOfScans=las_recv_len/sizeof(LaserData);
        //tu mame data..zavolame si funkciu

        //     memcpy(&sens,buff,sizeof(sens));
        int returnValue=autonomouslaser(measure);

        if(applyDelay==true)
        {
            struct timespec t;
            LidarVector newcommand;
            memcpy(&newcommand.data,&measure,sizeof(LaserMeasurement));
            //    clock_gettime(CLOCK_REALTIME,&t);
            newcommand.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
            auto timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
            lidarQuerry.push_back(newcommand);
            for(int i=0;i<lidarQuerry.size();i++)
            {
                if((std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp-lidarQuerry[i].timestamp)).count()>(2.5*1000000000))
                {
                    returnValue=locallaser(lidarQuerry[i].data);
                    if(returnValue!=-1)
                    {
                        //sendRobotCommand(returnValue);
                    }
                    lidarQuerry.erase(lidarQuerry.begin()+i);
                    i--;
                    break;

                }
            }

        }
        else
        {


            returnValue=locallaser(measure);
            if(returnValue!=-1)
            {
                //sendRobotCommand(returnValue);
            }
        }
    }
    std::cout<<"koniec thread"<<std::endl;
}

void MainWindow::sendRobotCommand(char command,double speed,int radius)
{
    {

        std::vector<unsigned char> mess;
        switch(command)
        {
        case  ROBOT_VPRED:
            mess=robot.setTranslationSpeed(speed);
            break;
        case ROBOT_VZAD:
            mess=robot.setTranslationSpeed(speed);
            break;
        case ROBOT_VLAVO:
            mess=robot.setRotationSpeed(speed);
            break;
        case ROBOT_VPRAVO:
            mess=robot.setRotationSpeed(speed);
            break;
        case ROBOT_STOP:
            mess=robot.setTranslationSpeed(0);
            break;
        case ROBOT_ARC:
            mess=robot.setArcSpeed(speed,radius);
            break;


        }
        if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
        {

        }
    }
  /*  else
    {
        struct timespec t;
        RobotCommand newcommand;
        newcommand.command=command;
        newcommand.radius=radius;
        newcommand.speed=speed;
        //clock_gettime(CLOCK_REALTIME,&t);
        newcommand.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
        commandQuery.push_back(newcommand);
    }*/
}

void MainWindow::autonomousRobotCommand(char command,double speed,int radius)
{
    return;
    std::vector<unsigned char> mess;
    switch(command)
    {
    case  ROBOT_VPRED:
        mess=robot.setTranslationSpeed(speed);
        break;
    case ROBOT_VZAD:
        mess=robot.setTranslationSpeed(speed);
        break;
    case ROBOT_VLAVO:
        mess=robot.setRotationSpeed(speed);
        break;
    case ROBOT_VPRAVO:
        mess=robot.setRotationSpeed(speed);
        break;
    case ROBOT_STOP:
        mess=robot.setTranslationSpeed(0);
        break;
    case ROBOT_ARC:
        mess=robot.setArcSpeed(speed,radius);
        break;

    }
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::robotexec()
{


    if(applyDelay==true)
    {
        struct timespec t;

        // clock_gettime(CLOCK_REALTIME,&t);
        auto timestamp=std::chrono::steady_clock::now();;//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
        for(int i=0;i<commandQuery.size();i++)
        {
       //     std::cout<<(std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp-commandQuery[i].timestamp)).count()<<std::endl;
            if((std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp-commandQuery[i].timestamp)).count()>(2.5*1000000000))
            {
                char cmd=commandQuery[i].command;
                std::vector<unsigned char> mess;
                switch(cmd)
                {
                case  ROBOT_VPRED:
                    mess=robot.setTranslationSpeed(commandQuery[i].speed);
                    break;
                case ROBOT_VZAD:
                    mess=robot.setTranslationSpeed(commandQuery[i].speed);
                    break;
                case ROBOT_VLAVO:
                    mess=robot.setRotationSpeed(commandQuery[i].speed);
                    break;
                case ROBOT_VPRAVO:
                    mess=robot.setRotationSpeed(commandQuery[i].speed);
                    break;
                case ROBOT_STOP:
                    mess=robot.setTranslationSpeed(0);
                    break;
                case ROBOT_ARC:
                    mess=robot.setArcSpeed(commandQuery[i].speed,commandQuery[i].radius);
                    break;

                }
                if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
                {

                }
                commandQuery.erase(commandQuery.begin()+i);
                i--;

            }
        }
    }
}

void MainWindow::paintThisLidar(LaserMeasurement &laserData)
{
    memcpy( &paintLaserData,&laserData,sizeof(LaserMeasurement));
    updateLaserPicture=1;
    update();
}

void MainWindow::on_pushButton_9_clicked()//left
{
    CommandVector help;
    help.command.commandType=2;
    help.command.actualAngle=0;
    help.command.actualDist=0;
    help.command.desiredAngle=20;
    help.command.desiredDist=0;
    struct timespec t;

    //   clock_gettime(CLOCK_REALTIME,&t);
    help.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
    AutonomousCommandQuerry.push_back(help);
}

void MainWindow::skeletonprocess()
{

    std::cout<<"init skeleton"<<std::endl;
#ifdef _WIN32
    WSADATA wsaData = {0};
    int iResult = 0;
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
#else
#endif
    ske_slen = sizeof(ske_si_other);
    if ((ske_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    char ske_broadcastene=1;
#ifdef _WIN32
    DWORD timeout=100;

    std::cout<<setsockopt(ske_s, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof timeout)<<std::endl;
    std::cout<<setsockopt(ske_s,SOL_SOCKET,SO_BROADCAST,&ske_broadcastene,sizeof(ske_broadcastene))<<std::endl;
#else
    setsockopt(ske_s,SOL_SOCKET,SO_BROADCAST,&ske_broadcastene,sizeof(ske_broadcastene));
#endif
    // zero out the structure
    memset((char *) &ske_si_me, 0, sizeof(ske_si_me));

    ske_si_me.sin_family = AF_INET;
    ske_si_me.sin_port = htons(23432);
    ske_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    ske_si_posli.sin_family = AF_INET;
    ske_si_posli.sin_port = htons(23432);
    ske_si_posli.sin_addr.s_addr = inet_addr(ipaddress.data());;//htonl(INADDR_BROADCAST);
    std::cout<<::bind(ske_s , (struct sockaddr*)&ske_si_me, sizeof(ske_si_me) )<<std::endl;;
    char command=0x00;

    skeleton bbbk;
    double measure[225];
    while(stopall==1)
    {

        if ((ske_recv_len = ::recvfrom(ske_s, (char *)&bbbk.joints, sizeof(char)*1800, 0, (struct sockaddr *) &ske_si_other, &ske_slen)) == -1)
        {

        //    std::cout<<"problem s prijatim"<<std::endl;
            continue;
        }


        memcpy(kostricka.joints,bbbk.joints,1800);
     updateSkeletonPicture=1;
  //      std::cout<<"doslo "<<ske_recv_len<<std::endl;
      //  continue;
        for(int i=0;i<75;i+=3)
        {
        //    std::cout<<klby[i]<<" "<<bbbk.joints[i].x<<" "<<bbbk.joints[i].y<<" "<<bbbk.joints[i].z<<std::endl;
        }
    }
    std::cout<<"koniec thread"<<std::endl;
}

void MainWindow::on_checkBox_2_clicked(bool checked)
{
    showLidar=checked;
}


void MainWindow::on_checkBox_3_clicked(bool checked)
{
    showCamera=checked;
}


void MainWindow::on_checkBox_4_clicked(bool checked)
{
    showSkeleton=checked;
}

void MainWindow::imageViewer()
{
    cv::VideoCapture cap;
    cap.open("http://127.0.0.1:8889/stream.mjpg");
    cv::Mat frameBuf;
    while(1)
    {
        cap >> frameBuf;


        if(frameBuf.rows<=0)
        {
            std::cout<<"nefunguje"<<std::endl;
            continue;
        }

        if(applyDelay==true)
        {
            struct timespec t;
            CameraVector newcommand;
            frameBuf.copyTo(newcommand.data);
            //    clock_gettime(CLOCK_REALTIME,&t);
            newcommand.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
            auto timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
            cameraQuerry.push_back(newcommand);
            for(int i=0;i<cameraQuerry.size();i++)
            {
                if((std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp-cameraQuerry[i].timestamp)).count()>(2.5*1000000000))
                {

                    cameraQuerry[i].data.copyTo(robotPicture);
                    cameraQuerry.erase(cameraQuerry.begin()+i);
                    i--;
                    break;

                }
            }

        }
        else
        {


           frameBuf.copyTo(robotPicture);
        }
        frameBuf.copyTo(AutonomousrobotPicture);
        updateCameraPicture=1;

        update();
        cv::waitKey(1);
        QCoreApplication::processEvents();
    }
}

// 640 x 480 (collumns x rows)
bool isValid(uchar* screen, int m, int n, int x, int y, int prevC, int newC)
{
    if(x < 0 || x >= m || y < 0 || y >= n || screen[x*640 + y] != prevC
       || screen[x*640 + y]== newC)
        return false;
    return true;
}

double MainWindow::RadToDegree(double radians)
{
    return radians * (180/PI);
}

double MainWindow::DegreeToRad(double degrees)
{
    return degrees * (PI/180);
}
