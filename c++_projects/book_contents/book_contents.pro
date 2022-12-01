TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    book.cpp \
    cli.cpp \
    utils.cpp

HEADERS += \
    book.hh \
    cli.hh \
    utils.hh

