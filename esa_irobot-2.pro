#-------------------------------------------------
#
# Project created by QtCreator 2017-06-30T16:42:47
#
#-------------------------------------------------

QT       += core gui
CONFIG += c++14
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = esa_irobot
TEMPLATE = app
win32 {
    INCLUDEPATH += C:/opencv_vc16/include/

    LIBS +=-LC:/opencv_vc16/bin
    LIBS +=-LC:/opencv_vc16/lib

    CONFIG(debug, debug|release) {
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_core440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_highgui440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_imgcodecs440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_imgproc440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_features2d440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_calib3d440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_videoio440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_ml440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_dnn440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_flann440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_objdetect440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_photo440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_shape440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_video440d
    }
    else {
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_core440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_highgui440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_imgcodecs440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_imgproc440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_features2d440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_calib3d440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_videoio440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_ml440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_dnn440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_flann440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_objdetect440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_photo440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_shape440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_video440
    }
}
unix {
#    PKGCONFIG += opencv4

    INCLUDEPATH += $$(HOME)/.local/lidaretto/include/opencv4/
    LIBS += -L$$(HOME)/.local/lidaretto/lib/        \
        -l:libopencv_core.so.4.5       \
        -l:libopencv_highgui.so.4.5    \
        -l:libopencv_imgcodecs.so.4.5  \
        -l:libopencv_imgproc.so.4.5    \
        -l:libopencv_features2d.so.4.5 \
        -l:libopencv_calib3d.so.4.5    \
        -l:libopencv_videoio.so.4.5    \
        -l:libopencv_ml.so.4.5         \
        -l:libopencv_dnn.so.4.5        \
        -l:libopencv_flann.so.4.5      \
        -l:libopencv_objdetect.so.4.5  \
        -l:libopencv_photo.so.4.5      \
        -l:libopencv_shape.so.4.5      \
        -l:libopencv_video.so.4.5
}

win32 {
LIBS += -lws2_32
LIBS += -lWinmm
}
SOURCES += main.cpp\
        mainwindow.cpp \
    rplidar.cpp \
    CKobuki.cpp

HEADERS  += mainwindow.h \
    rplidar.h \
    CKobuki.h

FORMS    += mainwindow.ui
