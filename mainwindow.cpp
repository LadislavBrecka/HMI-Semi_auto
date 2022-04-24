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

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setMouseTracking(true);

    showCamera=false;
    showLidar=true;
    showSkeleton=false;
    applyDelay=true;

    dl=0;
    stopall=1;
    updateCameraPicture=0;

    ipaddress="127.0.0.1";
    std::function<void(void)> f =std::bind(&robotUDPVlakno, (void *)this);
    robotthreadHandle=std::thread(f);
    std::function<void(void)> f2 =std::bind(&laserUDPVlakno, (void *)this);
    laserthreadHandle=std::thread(f2);

    std::function<void(void)> f3 =std::bind(&skeletonUDPVlakno, (void *)this);
    skeletonthreadHandle=std::thread(f3);

    QFuture<void> future = QtConcurrent::run([=]() {
        imageViewer();
        // Code in this block will run in another thread
    });

    Imager.start();
}

// funkcia local robot je na priame riadenie robota, ktory je vo vasej blizskoti,
// viete si z dat ratat polohu atd (zapnutie dopravneho oneskorenia sposobi
// opozdenie dat oproti aktualnemu stavu robota)
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

        pa1 = 0.06;
        pa2 = 0.12;
        pd = 0.035f;

        // regulator setup
        P_distance = 200.0f;
        P_angle = 0.25f;
        speedDifferenceLimit = MAX_START_SPEED;
        speedLimit = MAX_SPEED_LIMIT;

        // prevision values setup
        prevRotSpeed = rotSpeed = 0.0f;
        prevTransSpeed = transSpeed = 0.0f;

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

    // REGULATION
    if (!fifoTargets.GetPoints().empty() && (navigate || 0))
    {
        // first in, first out -> dat mi prvy element zo zasobnika
        Point target = fifoTargets.Out();
        Point actual(x, y);
        // ziskanie chyby polohy
        auto targetOffset = GetTargetOffset(actual, target);
        // regulacia rotacie
        RegulatorRotation(targetOffset.second);
        // ak nerotujeme, regulujeme doprednu rychlost
        RegulatorTranslation(targetOffset.first, targetOffset.second);
        // vyhodnotenie, ci splname poziadavky polohy
        EvaluateRegulation(targetOffset.first, targetOffset.second);
    }

    // CREATING MAP
    if (!isRotating && map_mode)
    {
        for(int k=0;k<paintLaserData.numberOfScans/*360*/;k++)
        {
            double dist  = paintLaserData.Data[k].scanDistance;

            if (dist > 150.0f && dist < 2500.0f)
            {
                double angle = paintLaserData.Data[k].scanAngle;

                double angle_sum = f_k + DegreeToRad(360.0f - angle);

                if (angle_sum >= 2*PI)
                    angle_sum -= 2*PI;
                else if (angle_sum < 0.0f)
                    angle_sum += 2*PI;

                double x_lidar = x + (dist / 1000.0f) * cos(angle_sum);
                double y_lidar = y + (dist / 1000.0f) * sin(angle_sum);

                map.setWall(Point(x_lidar, y_lidar));
            }
        }

        float rounded_x = (int)(x * 100 + .5); rounded_x = (float)rounded_x / 100;
        float rounded_y = (int)(y * 100 + .5); rounded_y = (float)rounded_y / 100;
        trajectory.insert(std::make_unique<Point>(rounded_x, rounded_y));
    }

    if (l_r != l_l)
        isRotating = true;
    else
        isRotating = false;
}

//sposob kreslenia na obrazovku, tento event sa spusti vzdy ked sa bud zavola funkcia update() alebo operacny system si vyziada prekreslenie okna
void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setBrush(Qt::gray);
    QPen pero;
    pero.setStyle(Qt::SolidLine);
    pero.setWidth(3);
    pero.setColor(Qt::green);

    mainRect   = ui->mainFrame->geometry();
    cameraRect = ui->cameraFrame->geometry();
    mapRect    = ui->mapFrame->geometry();

    painter.drawRect(mainRect);
    painter.setBrush(Qt::white);
    painter.drawRect(cameraRect);

    painter.setPen(QPen(Qt::black));
    painter.drawRect(mapRect);

    painter.setBrush(Qt::black);


    mainWidth  = mainRect.bottomRight().x() - mainRect.topLeft().x();
    mainHeight = mainRect.bottomRight().y() - mainRect.topLeft().y();

    // draw lidar integration
    painter.setPen(pero);

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
            if (D < 2.5f && D > 0.15)
            {
                float Y = 0.06f;
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

        // computing x and y for standalone lidar frame
        float zooming_x = 5000.0 / mainWidth;
        float zooming_y = 5000.0 / mainHeight;
        float dist_x = paintLaserData.Data[k].scanDistance / zooming_x;
        float dist_y = paintLaserData.Data[k].scanDistance / zooming_y;
        float xp   = mainRect.bottomRight().x() - (mainWidth  / 2.0 + dist_x * sin((360.0 - paintLaserData.Data[k].scanAngle) * 3.14159 / 180.0));
        float yp   = mainRect.bottomRight().y() - (mainHeight / 2.0 + dist_y * cos((360.0 - paintLaserData.Data[k].scanAngle) * 3.14159 / 180.0));

        if (draw_robot)
        {
            float xp   = mainRect.bottomRight().x() - (mainWidth  / 2.0 + 1.0 * sin((360.0 - 0.0) * 3.14159 / 180.0));
            float yp   = mainRect.bottomRight().y() - (mainHeight / 2.0 + 1.0 * cos((360.0 - 0.0) * 3.14159 / 180.0));

            painter.setPen(QPen(Qt::blue));
            painter.drawEllipse(QPointF(xp, yp), 6.0 * (mainWidth/640.0), 6.0 * (mainHeight/480.0));

            float xp_2 = xp + 0.0f * (mainWidth/640.0);
            float yp_2 = yp - 10.0f * (mainHeight/480.0);
            painter.drawLine(QLine(QPoint(xp, yp), QPoint(xp_2, yp_2)));

            xp -= 6.0 * (mainWidth/640.0);
            xp_2 = xp + 12.0 * (mainWidth/640.0);
            yp_2 = yp;
            painter.drawLine(QLine(QPoint(xp, yp), QPoint(xp_2, yp_2)));

            draw_robot = false;
        }

        // painting lidar points to standalone frame
        if(xp < mainRect.bottomRight().x() && xp > mainRect.topLeft().x() &&
           yp < mainRect.bottomRight().y() && yp > mainRect.topLeft().y() &&
           !((yp < mainHeight/20 + mainHeight/35 + mainHeight/70.0 + 15) && (xp < mainWidth/40 + (mainHeight/70.0) * 25))

          )
        {
            // change color according to distance
            painter.setPen(QPen(Qt::gray));
            painter.drawEllipse(QPointF(xp, yp), 2.0 * (mainWidth/640.0), 2.0 * (mainHeight/480.0));
        }
    }

    // draw main camera
    QImage imgIn = QImage((uchar*) robotPicture.data, robotPicture.cols, robotPicture.rows, robotPicture.step, QImage::Format_BGR888);
    painter.drawImage(cameraRect ,imgIn,imgIn.rect());

    // draw map
    double mapRectWidth  = mapRect.bottomRight().x() - mapRect.topLeft().x();
    double mapRectHeight = mapRect.bottomRight().y() - mapRect.topLeft().y();
    if (map.filled)
    {
        for (int i = 0; i < MAP_HEIGHT; ++i)
        {
            for (int j = 0; j < MAP_WIDTH; ++j)
            {
                if (map.map[i][j] == 1)
                {
                    float point_x = mapRect.topLeft().x() + (j * mapRectWidth)  / MAP_WIDTH;
                    float point_y = mapRect.topLeft().y() + (i * mapRectHeight) / MAP_HEIGHT;

                    painter.drawEllipse(QPointF(point_x, point_y), 2 * (mapRectWidth/500), 2 * (mapRectHeight/500));

                }
            }
        }
    }

    for (auto&& p : trajectory)
    {
        // make conversion from world coordinates to map frame coordinates
        int traj_x = round(p->x * MAP_STEP + MAP_WIDTH  / 2);
        int traj_y = MAP_HEIGHT - round(p->y * MAP_STEP + MAP_HEIGHT / 2);

        float point_x = mapRect.topLeft().x() + (traj_x * mapRectWidth)  / MAP_WIDTH;
        float point_y = mapRect.topLeft().y() + (traj_y * mapRectHeight) / MAP_HEIGHT;

        // change color according to distance
        painter.setPen(QPen(Qt::green));
        painter.setBrush(QBrush(Qt::green));
        painter.drawEllipse(QPointF(point_x, point_y), 1.4 * (mapRectWidth/500), 1.4 * (mapRectHeight/500));
    }

    // draw text to lidar frame
    painter.setPen(QPen(Qt::blue));
    painter.setFont(QFont("Times", (mainHeight/70.0), QFont::Bold));
    std::string message1 = "Trans. speed is: " + std::to_string(actualSpeed);
    painter.drawText(QPoint(mainWidth/40, mainHeight/20), message1.c_str());
    std::string message2 = "Command send to robot: " + std::string(DirectionToString(direction));
    painter.drawText(QPoint(mainWidth/40, mainHeight/20 + mainHeight/35), message2.c_str());
}

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

// Implement in your widget
void MainWindow::mousePressEvent(QMouseEvent *event){

    QPoint p = event->pos();

    mainWidth  = mainRect.bottomRight().x() - mainRect.topLeft().x();
    mainHeight = mainRect.bottomRight().y() - mainRect.topLeft().y();

    if (p.x() > mainRect.topLeft().x() && p.x() < mainRect.bottomRight().x() &&
        p.y() > mainRect.topLeft().y() && p.y() < mainRect.bottomRight().y() )
    {
        // convert to mainFrame coordinates
        double tx = p.x() - mainRect.topLeft().x();
        double ty = p.y() - mainRect.topLeft().y();

        // cast to 640x480
        tx = (tx * 640.0) / mainWidth;
        ty = 480.0 - (ty * 480.0) / mainHeight;

        // center point coordinates
        tx -= 640.0/2.0;
        ty -= 480.0/2.0;

        // tx is distance in x axis to point in pixels
        // ty is distance in y axis to point in pixels

        tx = (tx * 5000.0f) / 640.0;
        ty = (ty * 5000.0f) / 480.0;
        tx /= 1000.0f;
        ty /= 1000.0f;

        float theta = f_k - PI/2;

        float ox = 0;
        float oy = 0;

        double new_x = cos(theta) * (tx-ox) - sin(theta) * (ty-oy) + ox;
        double new_y = sin(theta) * (tx-ox) + cos(theta) * (ty-oy) + oy;

        double target_world_x = x + new_x;
        double target_world_y = y + new_y;

        // now tx is distance in x axis to point in meters
        // now ty is distance in y axis to point in meters

        // check with zone if point is reachable
        Point target(target_world_x, target_world_y);
        Point actual(x, y);
        // ziskanie chyby polohy
        auto targetOffset = GetTargetOffset(actual, target);

        bool unreachable = false;
        for(int k=0;k<paintLaserData.numberOfScans;k++)
        {
            float D = paintLaserData.Data[k].scanDistance / 1000.0;
            float a = 360.0 - paintLaserData.Data[k].scanAngle;
            if (a > 360)    a -= 360;
            if (a < 0)      a += 360;
            float pointAngleZone = DegreeToRad(a) - targetOffset.second;

            if (D > 0.005 && D < targetOffset.first && (pointAngleZone < PI/2 || pointAngleZone > (3/2)*PI) )
            {
                float dCrit = b / sin(pointAngleZone);
                if (dCrit > D)
                {
                    unreachable = true;
                    break;
                }
            }
        }

        std::cout << "Point is: [" << target_world_x << ", " << target_world_y << "]. " << std::endl;

        if (unreachable)
            std::cout << "Point is unreachable!" << std::endl;
        else
        {
            std::cout << "Point is reachable!" << std::endl;
            // add to queue
            fifoTargets.In(Point(target_world_x, target_world_y));

            // turn on navigation
            navigate = true;
        }
    }
}

void MainWindow::RobotSetTranslationSpeed(float speed)
{
    if (speed > 0.0)        direction = FORWARD;
    else if (speed < 0.0)   direction = BACKWARD;
    else                    direction = STOP;

    actualSpeed = speed;

    std::vector<unsigned char> mess = robot.setTranslationSpeed(speed);
    if (sendto(rob_s, (char*) mess.data(), sizeof(char) *mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::RobotSetRotationSpeed(float speed)
{
    if (speed > 0.0)        direction = LEFT;
    else if (speed < 0.0)   direction = RIGHT;
    else                    direction = STOP;

    std::vector<unsigned char> mess = robot.setRotationSpeed(speed);
    if (sendto(rob_s, (char*) mess.data(), sizeof(char) *mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

std::pair<double, double> MainWindow::GetTargetOffset(Point actual, Point target)
{
    float dx = target.x - actual.x;
    float dy = target.y - actual.y;
    double alfa;

    // ak je zmena v osi y nulova
    if (dy == 0.0f)
    {
        alfa = 0.0f;
    }
    //ak je zmena v osi x nulova
    else if (dx == 0.0f)
    {
        alfa = PI/2.0f;
    }
    // ak ani jeden predosli pripad nenastal, je delenie bezpecne
    else
    {
        alfa = atan(fabs(dy)/fabs(dx));
    }

    bool negativeX =  dx < 0.0f;
    bool negativeY =  dy < 0.0f;

    // umiestnenie uhla do spravneho kvadrantu -> theta e <0, 360>
    // 2. kvadrant
    if (negativeX && !negativeY)
    {
        alfa = PI - alfa;
    }
    // 3. kvadrant
    else if (negativeX && negativeY)
    {
       alfa = alfa + PI;
    }
    // 4. kvadrant
    else if (!negativeX && negativeY)
    {
       alfa = 2*PI - alfa;
    }

    float distance = sqrt(pow(dx, 2) + pow(dy, 2));
    double thetaOffset = 0.0f;
    if (f_k < alfa)
    {
        thetaOffset = alfa - f_k;
        if (thetaOffset > PI)
        {
            thetaOffset = 2*PI - thetaOffset;
            thetaOffset *= (-1);
        }
    }
    else if (f_k > alfa)
    {
        thetaOffset = f_k - alfa;
        if (thetaOffset <= PI)
        {
            thetaOffset *= (-1);
        }
        else if (thetaOffset > PI)
        {
            thetaOffset = 2*PI - thetaOffset;
        }
    }

   return std::pair<double, double>(distance, thetaOffset);
}

void MainWindow::RegulatorRotation(double dTheta)
{
    // ak je uhol vacsi ako mrtve pasmo pa2, regulator reguluje uhol natocenia
    if (fabs(dTheta) > pa2)
    {
        // P Regulator s rampou
        float idealSpeed = P_angle * dTheta;
        if (idealSpeed > prevRotSpeed + speedDifferenceLimit)
            rotSpeed += speedDifferenceLimit;
        else
            rotSpeed = idealSpeed;
    }

    // ak je uhol mansie ako vnutorne mrtve pasmo pa1, ukoncujeme regulaciu uhla natocenia
    else if (fabs(dTheta) < pa1)
    {
       rotSpeed = 0.0f;
    }

    if (rotSpeed > speedLimit)
        rotSpeed = speedLimit;

    RobotSetRotationSpeed(rotSpeed);
    prevRotSpeed = rotSpeed;
}

void MainWindow::RegulatorTranslation(double distance, double dTheta)
{
    // ak je robot spravne natoceny a neprebieha rotacia, regulator reguluje doprednu rychlost
    if (distance > pd && abs(dTheta) <= pa2)
    {
        float idealSpeed = P_distance * distance;

        if (idealSpeed > prevTransSpeed + speedDifferenceLimit)
            transSpeed += speedDifferenceLimit;
        else
            transSpeed = idealSpeed;

        if (transSpeed > speedLimit)
            transSpeed = speedLimit;

        RobotSetTranslationSpeed(transSpeed);
    }

    prevTransSpeed = transSpeed;
}

void MainWindow::EvaluateRegulation(double distance, double theta)
{
    if (distance <= pd)
    {
        // dosiahnutie ciela
        transSpeed = 0.0f;
        rotSpeed   = 0.0f;

        // tato funkcia nastavi 0 rychlost na obe kolesa
        RobotSetTranslationSpeed(transSpeed);

        fifoTargets.Pop();

        if (fifoTargets.GetPoints().empty())
        {

            navigate = false;
            std::cout << std::endl << "Point reached!" << std::endl;
        }
    }
}

double MainWindow::RadToDegree(double radians)
{
    return radians * (180/PI);
}

double MainWindow::DegreeToRad(double degrees)
{
    return degrees * (PI/180);
}

void MainWindow::PrintTargetQueue()
{
    std::vector<Point> targets = fifoTargets.GetPoints();

    std::string message;

    if (!targets.empty())
    {
        for (auto v : targets)
                message += " --  X: " + std::to_string(v.x) + ", Y:" + std::to_string(v.y) + " --  ";
    }

    std::cout << message << std::endl;
}

void MainWindow::on_mapButton_clicked(bool checked)
{
    if (checked)
    {
        speedLimit = 150.0f;
        speedDifferenceLimit = 25.0;
        map_mode = true;
    }
    else
    {
        map_mode = false;
        speedLimit = MAX_SPEED_LIMIT;
        speedDifferenceLimit = MAX_START_SPEED;
    }
}

void MainWindow::on_saveMap_clicked(bool checked)
{
    map.saveToFile();
}


//--cokolvek za tymto vas teoreticky nemusi zaujimat, su tam len nejake skarede kody

// funkcia local laser je naspracovanie dat z lasera(zapnutie dopravneho oneskorenia sposobi
// opozdenie dat oproti aktualnemu stavu robota)
int MainWindow::locallaser(LaserMeasurement &laserData)
{
    paintThisLidar(laserData);
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
                newcommand.timestamp = std::chrono::steady_clock::now();;//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
                auto timestamp = std::chrono::steady_clock::now();;//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
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

void MainWindow::on_safeStop_clicked(bool checked)
{
    RobotSetTranslationSpeed(0.0f);
}

