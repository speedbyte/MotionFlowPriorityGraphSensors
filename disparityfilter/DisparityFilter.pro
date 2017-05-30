#-------------------------------------------------
#
# Project created by QtCreator 2013-10-21T17:21:47
#
#-------------------------------------------------

TARGET = DisparityFilter
TEMPLATE = lib
CONFIG += plugin
CONFIG -= flat
QT += core

SOURCES += \
    DisparityRefinementFilter.cpp \
    SGBMFilter.cpp \
    DisparityToDepthFilter.cpp \

HEADERS += \
    SGBMFilter.h \
    DisparityToDepthFilter.h \
    DisparityRefinementFilter.h\
    Container.h


include(../commonPlugin.pri)
include(../opencv_libs.pri)
include(../boost_libs.pri)