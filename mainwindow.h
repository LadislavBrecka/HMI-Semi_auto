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
#include <iostream>
#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include <set>
#include <array>
#include <queue>
#include <algorithm>
#include <chrono>
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
#include <QTextStream>
#include <QKeyEvent>
#define ROBOT_VPRED 0x01
#define ROBOT_VZAD 0x02
#define ROBOT_VLAVO 0x04
#define ROBOT_VPRAVO 0x03
#define ROBOT_STOP 0x00
#define ROBOT_ARC 0x05

#define MAX_SPEED_LIMIT 200
#define MAX_START_SPEED 50

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

struct Point {
    float x;
    float y;

private:
    float m_invalidX = 9999.0;
    float m_invalidY = 9999.0;

public:
    Point(float setX, float setY)
    {
        x = setX;
        y = setY;
    }

    Point()
    {
        x = m_invalidX;
        y = m_invalidY;
    }

    bool isValid()
    {
        if (x == m_invalidX && y == m_invalidY)
            return false;
        else
            return true;
    }
};

#define MAP_WIDTH 144
#define MAP_HEIGHT 144
#define MAP_STEP 12

class Map
{
public:
   int map[MAP_WIDTH][MAP_HEIGHT];
   bool filled = false;

public:
    Map()
    {
        for(int i = 0; i < MAP_HEIGHT; i++)
        {
            for(int j = 0; j < MAP_WIDTH; j++)
            {
                map[i][j] = 0;
            }
        }

        map[MAP_WIDTH/2][MAP_HEIGHT/2] = -1;
    }

    void setWall(Point lidar)
    {
        if (!filled) filled = true;
        int x = round(lidar.x * MAP_STEP + MAP_WIDTH  / 2);
        int y = MAP_HEIGHT - round(lidar.y * MAP_STEP + MAP_HEIGHT / 2);
        map[y][x] = 1;
    }


    void saveToFile(std::string filename="map.txt")
    {
        QFile file(filename.c_str());
        file.open(QIODevice::WriteOnly | QIODevice::Text);
        QTextStream out(&file);

        for (int i = 0; i < MAP_HEIGHT; ++i)
        {
            for (int j = 0; j < MAP_WIDTH; ++j)
            {
                if (map[i][j] == 1)
                    out << "*";
                else
                    out << " ";
            }
            out << "\n";
        }
        std::cout << "Map saved to file " << filename << "!" << std::endl;
        file.close();
    }

    void loadFromFile(std::string filename)
    {
        QFile file(filename.c_str());
        file.open(QIODevice::ReadOnly | QIODevice::Text);
        QTextStream in(&file);
        char temp;
        for (int i = 0; i < MAP_HEIGHT; ++i)
        {
            for (int j = 0; j < MAP_WIDTH; ++j)
            {
                in >> temp;
                if (temp == '*')
                    map[i][j] = 1;
                else if (temp == ' ')
                    map[i][j] = 0;
            }
            in >> temp;
        }

        std::cout << "Map load from file " << filename << "!" << std::endl;
    }

    void saveToFileRaw(std::string filename="map_raw.txt")
    {
        QFile file(filename.c_str());
        file.open(QIODevice::WriteOnly | QIODevice::Text);
        QTextStream out(&file);

        for (int i = 0; i < MAP_HEIGHT; ++i)
        {
            for (int j = 0; j < MAP_WIDTH; ++j)
            {
                if (map[i][j] < 10)
                    out << "  " << map[i][j] << " ";
                else if (map[i][j] < 100)
                    out << " " << map[i][j] << " ";
                else
                    out << map[i][j];
            }
            out << "\n";
        }
        std::cout << "Raw map saved to file " << filename << "!" << std::endl;
        file.close();
    }

};

class FifoQueue {

private:
    std::vector<Point> m_targets;

public:

    void In(Point p)
    {
        m_targets.push_back(p);
    }

    Point Out()
    {
        if (!m_targets.empty())
        {
            return m_targets.front();;
        }
        else
            return Point();
    }

    void Pop()
    {
        if (!m_targets.empty())
        {
            m_targets.erase(m_targets.begin());
        }
    }

    std::vector<Point> GetPoints()
    {
        return m_targets;
    }
};

enum Direction
{
    STOP=0,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

constexpr const char* DirectionToString(Direction e) noexcept
{
    switch (e)
    {
        case Direction::STOP: return "STOP";
        case Direction::FORWARD: return "FORWARD";
        case Direction::BACKWARD: return "BACKWARD";
        case Direction::LEFT: return "LEFT";
        case Direction::RIGHT: return "RIGHT";
        default: return "NOT DEFINED";
    }
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    QRect mainRect;
    QRect cameraRect;
    QRect mapRect;

    int mainWidth;
    int mainHeight;

    int mapWidth;
    int mapHeight;

    Direction direction = STOP;

    // utility funkcie
    std::pair<double, double> GetTargetOffset(Point actual, Point target);
    double RadToDegree(double radians);
    double DegreeToRad(double degrees);
    void   PrintTargetQueue();
    void   mapping();

    // kontrolne funkcie
    void RobotSetTranslationSpeed(float speed);
    void RobotSetRotationSpeed(float speed);
    void RegulatorRotation(double dTheta);
    void RegulatorTranslation(double distance, double dTheta);
    void EvaluateRegulation(double distance, double dTheta);

    // switche
    bool initParam = true;
    bool navigate = false;
    bool map_mode = true;
    bool traj_mode = true;
    bool isRotating = false;

    // signalizacie
    bool rotationLock;
    int rotationDir;

    // lokalizacia - stavove premenne
    double l_r, l_r_prev, l_l, l_l_prev, l_k;
    double x, x_prev, y, y_prev;
    double f_k, f_k_prev, d_alfa;
    unsigned int enc_l_prev, enc_r_prev;
    double total_l, total_r;

    // konstanty
    long const double tickToMeter = 0.000085292090497737556558; // [m/tick]
    long const double b = 0.23; // wheelbase distance in meters

    double actualSpeed; // aktualna rychlost poslana do robota
    float pa1, pa2;     // hodnoty mrtvych pasiem
    double epsilon;     // dead zone pre porovnanie s nulou
    double pd;          // mrtve pasmo pre distance

    std::set<std::unique_ptr<Point>> trajectory;    // kontainer trajektorii
    FifoQueue fifoTargets;                          // fifo queue pre target pozicie;
    Map map;                                        // mapa

    // regulator - parametre
    float P_distance;
    float P_angle;

    float speedDifferenceLimit;
    float speedLimit;

    float prevRotSpeed, rotSpeed;
    float prevTransSpeed, transSpeed;

    double pixelToMeter_x;
    double pixelToMeter_y;

    std::string ipaddress;
    std::vector<RobotCommand> commandQuery;
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    // premenne demo programu
    int dl;
    double zX;
    double zY;
    bool naviguj;
    int stopall;
    int robotthreadID;              // id vlakna
    double toleranciaUhla;
    std::thread robotthreadHandle;

    int updateLaserPicture;
    int updateCameraPicture;
    int updateSkeletonPicture;

    static void *robotUDPVlakno(void *param)
    {
        ((MainWindow*)param)->robotprocess();
        return 0;
    }

    int laserthreadID;  // id vlakna
    std::thread laserthreadHandle;

    static void *laserUDPVlakno(void *param)
    {
        ((MainWindow*)param)->laserprocess();
        return 0;
    }

    int skeletonthreadID;  // id vlakna
    std::thread skeletonthreadHandle;
    static void *skeletonUDPVlakno(void *param)
    {
        ((MainWindow*)param)->skeletonprocess();
        return 0;
    }

    QThread Imager;
    void imageViewer();
    void mousePressEvent(QMouseEvent *event) override;
    void sendRobotCommand(char command,double speed=0,int radius=0);
    void autonomousRobotCommand(char command,double speed=0,int radius=0);

    void robotexec();
    void robotprocess();
    void laserprocess();
    void skeletonprocess();
    void localrobot(TKobukiData &sens);
    void autonomousrobot(TKobukiData &sens);
    void paintThisLidar(LaserMeasurement &laserData);
    int  locallaser(LaserMeasurement &laserData);
    int  autonomouslaser(LaserMeasurement &laserData);

    LaserMeasurement paintLaserData;

    int las_s,  las_recv_len;
    struct sockaddr_in las_si_me, las_si_other,las_si_posli;

    int ske_s,  ske_recv_len;
    struct sockaddr_in ske_si_me, ske_si_other,ske_si_posli;

    int rob_s,  rob_recv_len;
    struct sockaddr_in rob_si_me, rob_si_other,rob_si_posli;

#ifdef _WIN32
    int rob_slen;
    int las_slen;
    int ske_slen;
#else
     unsigned int rob_slen;
     unsigned int las_slen;
     unsigned int ske_slen;
#endif
    QTimer *timer;
    CKobuki robot;
    TKobukiData sens;
    std::vector<RobotData> sensorQuerry;
    std::vector<LidarVector> lidarQuerry;
    std::vector<CameraVector> cameraQuerry;
    std::vector< CommandVector> AutonomousCommandQuerry;
    cv::Mat robotPicture;
    cv::Mat AutonomousrobotPicture;
    skeleton kostricka;

private slots:

    void on_trajButton_clicked(bool checked);
    void on_saveMap_clicked(bool checked);
    void on_safeStop_clicked(bool checked);

private:
    Ui::MainWindow *ui;
    bool showCamera;
    bool showLidar;
    bool showSkeleton;
    bool applyDelay;
    void paintEvent(QPaintEvent *event) override;
    void keyPressEvent(QKeyEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
};

#endif // MAINWINDOW_H
