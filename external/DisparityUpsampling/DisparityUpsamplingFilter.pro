#-------------------------------------------------
#
# Project created by QtCreator 2013-10-21T17:21:47
#
#-------------------------------------------------

TARGET = DisparityUpsamplingFilter
TEMPLATE = lib
CONFIG += plugin
CONFIG -= flat
QT += core

SOURCES += \
    DisparityUpsamplingFilter.cpp

HEADERS += \ 
    DisparityUpsamplingFilter.h \
    Container.h

OTHER_FILES += DisparityUpsamplingFilter.json

include(../commonPlugin.pri)
