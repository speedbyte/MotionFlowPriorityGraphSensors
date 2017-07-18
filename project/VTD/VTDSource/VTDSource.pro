#-------------------------------------------------
#
# Project created by QtCreator 2013-10-21T17:21:47
#
#-------------------------------------------------

TARGET = VTDSource
TEMPLATE = lib
CONFIG += plugin
CONFIG -= flat
QT += core

SOURCES += \
    VTDSource.cpp \
    RDBHandler.cc \
    VTDConnector.cpp \
    ../LaneDetectionApplication/LaneDetector.cc

HEADERS += \ 
    VTDSource.h \
    Container.h \
    viRDBIcd.h \
    RDBHandler.hh \
    ../server/Streaming/ImageUtils.h \
    ../server/PluginChain/TrafficSigns.h \
    ../server/PluginChain/Lanes.h \
    VTDConnector.h \
    ../LaneDetectionApplication/LaneDetector.hh

include(../commonPlugin.pri)
include(../boost_libs.pri)
include(../opencv_libs.pri)
