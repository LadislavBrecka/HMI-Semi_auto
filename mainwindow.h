#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#ifdef _WIN32
    #include<windows.h>
#else
    #include <termios.h>
    #include <unistd.h>
    #include "unistd.h"
    #include<arpa/inet.h>
    #include<unistd.h>
    #include<sys/socket.h>
#endif

#include<iostream>
#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<vector>
#include<set>
#include<array>
#include<algorithm>
#include<chrono>
#include "CKobuki.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"
#include "rplidar.h"
#include <QThread>
#include <QKeyEvent>
#define ROBOT_VPRED 0x01
#define ROBOT_VZAD 0x02
#define ROBOT_VLAVO 0x04
#define ROBOT_VPRAVO 0x03
#define ROBOT_STOP 0x00
#define ROBOT_ARC 0x05

static const char *klby[]={{"left_wrist"},{"left_thumb_cmc"},{"left_thumb_mcp"},{"left_thumb_ip"},{"left_thumb_tip"},{"left_index_cmc"},{"left_index_mcp"},{"left_index_ip"},{"left_index_tip"},{"left_middle_cmc"},{"left_middle_mcp"},{"left_middle_ip"},{"left_middle_tip"},{"left_ring_cmc"},{"left_ring_mcp"},{"left_ring_ip"},{"left_ringy_tip"},{"left_pinky_cmc"},{"left_pink_mcp"},{"left_pink_ip"},{"left_pink_tip"},{"right_wrist"},{"right_thumb_cmc"},{"right_thumb_mcp"},{"right_thumb_ip"},{"right_thumb_tip"},{"right_index_cmc"},{"right_index_mcp"},{"right_index_ip"},{"right_index_tip"},{"right_middle_cmc"},{"right_middle_mcp"},{"right_middle_ip"},{"right_middle_tip"},{"right_ring_cmc"},{"right_ring_mcp"},{"right_ring_ip"},{"right_ringy_tip"},{"right_pinky_cmc"},{"right_pink_mcp"},{"right_pink_ip"},{"right_pink_tip"},{"nose"},{"left_eye_in"},{"left_eye"},{"left_eye_out"},{"right_eye_in"},{"right_eye"},{"right_eye_out"},{"left_ear"},{"right_ear"},{"mounth_left"},{"mounth_right"},{"left_shoulder"},{"right_shoulder"},{"left_elbow"},{"right_elbow"},{"left_wrist"},{"right_wrist"},{"left_pinky"},{"right_pinky"},{"left_index"},{"right_index"},{"left_thumb"},{"right_thumb"},{"left_hip"},{"right_hip"},{"left_knee"},{"right_knee"},{"left_ankle"},{"right_ankle"},{"left_heel"},{"righ_heel"},{"left+foot_index"},{"right_foot_index"}};
enum jointnames
{
   left_wrist,
   left_thumb_cmc,
   left_thumb_mcp,
   left_thumb_ip,
   left_thumb_tip,
   left_index_cmc,
   left_index_mcp,
   left_index_ip,
   left_index_tip,
   left_middle_cmc,
   left_middle_mcp,
   left_middle_ip,
   left_middle_tip,
   left_ring_cmc,
   left_ring_mcp,
   left_ring_ip,
   left_ringy_tip,
   left_pinky_cmc,
   left_pink_mcp,
   left_pink_ip,
   left_pink_tip,
   right_wrist,
   right_thumb_cmc,
   right_thumb_mcp,
   right_thumb_ip,
   right_thumb_tip,
   right_index_cmc,
   right_index_mcp,
   right_index_ip,
   right_index_tip,
   right_middle_cmc,
   right_middle_mcp,
   right_middle_ip,
   right_middle_tip,
   right_ring_cmc,
   right_ring_mcp,
   right_ring_ip,
   right_ringy_tip,
   right_pinky_cmc,
   right_pink_mcp,
   right_pink_ip,
   right_pink_tip,
   nose,left_eye_in,
   left_eye,
   left_eye_out,
   right_eye_in,
   right_eye,
   right_eye_out,
   left_ear,
   right_ear,
   mounth_left,
   mounth_right,
   left_shoulder,
   right_shoulder,
   left_elbow,
   right_elbow,
   left_wrist_glob,
   right_wrist_glob,
   left_pinky,
   right_pinky,
   left_index,
   right_index,
   left_thumb,
   right_thumb,
   left_hip,
   right_hip,
   left_knee,
   right_knee,
   left_ankle,
   right_ankle,
   left_heel,
   righ_heel,
   left_foot_index,
   right_foot_index
};

typedef struct
{
    double x;
    double y;
    double z;
} klb;

typedef struct
{
    klb joints[75];
} skeleton;

typedef struct
{
    std::chrono::steady_clock::time_point timestamp;
    int command;
    double speed;
    int radius;
} RobotCommand;

typedef struct
{
    std::chrono::time_point<std::chrono::steady_clock> timestamp;
    TKobukiData sens;
} RobotData;

typedef struct
{
    int commandType;//0 ziaden, 1 pohyb, 2 uhol
     int desiredDist;
     int actualDist;
     int desiredAngle;
     int actualAngle;
} AutonomousCommand;

typedef struct
{
    std::chrono::steady_clock::time_point timestamp;
    AutonomousCommand command;
} CommandVector;

typedef struct
{
    std::chrono::steady_clock::time_point timestamp;
    LaserMeasurement data;
} LidarVector;


typedef struct
{
    std::chrono::steady_clock::time_point timestamp;
    cv::Mat data;
} CameraVector;

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    int direction = 0;
    void RobotSetTranslationSpeed(float speed);
    void RobotSetRotationSpeed(float speed);


    double RadToDegree(double radians);
    double DegreeToRad(double degrees);

    // switche
    bool initParam = true;
    int safe_stop = 0;

    // lokalizacia - stavove premenne
    double l_r, l_r_prev, l_l, l_l_prev, l_k;
    double x, x_prev, y, y_prev;
    double f_k, f_k_prev, d_alfa;
    unsigned int enc_l_prev, enc_r_prev;
    double total_l, total_r;
    long double tickToMeter = 0.000085292090497737556558; // [m/tick]
    long double b = 0.23; // wheelbase distance in meters, from kobuki manual https://yujinrobot.github.io/kobuki/doxygen/enAppendixProtocolSpecification.html
    bool use_skeleton = false;

    std::string ipaddress;
    std::vector<RobotCommand> commandQuery;
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    bool naviguj;
    double zX;
    double zY;
    double toleranciaUhla;
    int dl;
    int stopall;
    std::thread robotthreadHandle;
    int robotthreadID;  // id vlakna

    static void *robotUDPVlakno(void *param)
    {
        ((MainWindow*)param)->robotprocess();
        return 0;
    }

    std::thread laserthreadHandle;
    int laserthreadID;  // id vlakna

    static void *laserUDPVlakno(void *param)
    {
        ((MainWindow*)param)->laserprocess();

        return 0;
    }

    std::thread skeletonthreadHandle;
    int skeletonthreadID;  // id vlakna
    static void *skeletonUDPVlakno(void *param)
    {
        std::cout<<"startujem ci co"<<std::endl;
        ((MainWindow*)param)->skeletonprocess();
        return 0;
    }

    QThread Imager;
    void imageViewer();
    void sendRobotCommand(char command,double speed=0,int radius=0);
    void autonomousRobotCommand(char command,double speed=0,int radius=0);

    void robotexec();
    void robotprocess();
    void laserprocess();
    void skeletonprocess();
    void localrobot(TKobukiData &sens);
    void autonomousrobot(TKobukiData &sens);

    int locallaser(LaserMeasurement &laserData);
    int autonomouslaser(LaserMeasurement &laserData);

    void paintThisLidar(LaserMeasurement &laserData);
    LaserMeasurement paintLaserData;
    int updateLaserPicture;
    int updateCameraPicture;
    int updateSkeletonPicture;
    struct sockaddr_in las_si_me, las_si_other,las_si_posli;
    int las_s,  las_recv_len;
    struct sockaddr_in ske_si_me, ske_si_other,ske_si_posli;
    int ske_s,  ske_recv_len;
    struct sockaddr_in rob_si_me, rob_si_other,rob_si_posli;
    int rob_s,  rob_recv_len;

    #ifdef _WIN32
        int rob_slen;
        int las_slen;
        int ske_slen;
    #else
         unsigned int rob_slen;
         unsigned int las_slen;
         unsigned int ske_slen;
    #endif
    CKobuki robot;
    TKobukiData sens;
    QTimer *timer;
    std::vector<RobotData> sensorQuerry;
    std::vector<LidarVector> lidarQuerry;
    std::vector<CameraVector> cameraQuerry;
    std::vector< CommandVector> AutonomousCommandQuerry;
    cv::Mat robotPicture;
    cv::Mat AutonomousrobotPicture;
    skeleton kostricka;

private slots:

    void on_pushButton_9_clicked();
    void on_checkBox_2_clicked(bool checked);
    void on_checkBox_3_clicked(bool checked);
    void on_checkBox_4_clicked(bool checked);

private:
    bool showCamera;
    bool showLidar;
    bool showSkeleton;
    bool applyDelay;
    Ui::MainWindow *ui;
    void paintEvent(QPaintEvent *event);// Q_DECL_OVERRIDE;
    void keyPressEvent(QKeyEvent* event);
};

#endif // MAINWINDOW_H
