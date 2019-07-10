#-------------------------------------------------
#
# Project created by QtCreator 2016-11-07T14:38:11
#
#-------------------------------------------------

QT += core gui
QT += serialport
QT += network
QT += multimedia

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = BluHubClient
TEMPLATE = app

win32|win64 {
    DESTDIR = $$PWD/../Windows
}

macx {
    DESTDIR = $$PWD/../OSX
}

SOURCES += main.cpp\
        mainwindow.cpp \
        audio_src.cpp \
        device_manager.cpp \
    ../host/app_host/app_host.c \
    ../host/app_host/app_host_ag.c \
    ../host/wiced_hci/wcied_hci.c \
    ../host/wiced_hci/wcied_hci_ag.c \
    ../host/wiced_hci/wcied_hci_audio_src.c \
    ../host/app_host/app_host_audio_src.c \
    ../host/app_host/app_host_anp.c \
    ../host/wiced_hci/wcied_hci_anp.c \
    ../host/wiced_hci/wcied_hci_le_coc.c \
    recorder.cpp


unix:!macx {
    SOURCES += serial_port_linux.cpp
    SOURCES += btspy_ux.cpp
}

win32|win64 {
    SOURCES += btspy_win32.cpp
    SOURCES += serial_port_win32.cpp
}

macx {
    SOURCES += serial_port_mac.cpp
    SOURCES += btspy_ux.cpp
}


HEADERS  += mainwindow.h \
            avrc.h \
            serial_port.h \
            app_include.h \
    wiced_bt_defs.h \
    ../include/btle_homekit2_lightbulb.h \
    ../include/hci_control_api.h \
    ../host/app_host/app_host.h \
    ../host/wiced_hci/wiced_hci.h \
    ../host/wiced_hci/wiced_types.h

FORMS    += mainwindow.ui

INCLUDEPATH += ../include
INCLUDEPATH += ../host/app_host
INCLUDEPATH += ../host/wiced_hci

# ws2_32.lib and winmm.lib path might need to be adjusted on user PC, for example
# C:\WINDDK\7600.16385.0\lib\win7\i386\ws2_32.lib
# -L"Windows" -lws2_32
win32: LIBS += -lQt5Network ..\Windows\ws2_32.lib ..\Windows\winmm.lib

unix:!macx {
LIBS += -lasound -lrt
DEFINES += PCM_ALSA
}

RESOURCES     = resources.qrc

RC_ICONS = CY_Logo.ico

ICON = CY_Logo.icns

DISTFILES += \
    ../README.txt
