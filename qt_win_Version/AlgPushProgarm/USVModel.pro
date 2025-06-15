QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle
CONFIG += c++17


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/lib/ -lCUSVModel
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/lib/ -lCUSVModel
else:unix: LIBS += -L$$PWD/lib/ -lCUSVModel

INCLUDEPATH += $$PWD/.
DEPENDPATH += $$PWD/.

INCLUDEPATH += C:\\Users\\15205\\Desktop\\demo_qt_win_Version1121_last\\qt_win_Version\\AlgPushProgarm\\lib
LIBS += C:\\Users\\15205\\Desktop\\demo_qt_win_Version1121_last\\qt_win_Version\\AlgPushProgarm\\lib\\CUSVModel.dll


HEADERS += \
    cusvmodel.h \
    CUSVModel_global.h \
    cmultiusvmodel.h \
    CUSVCommon.h \
    Share.h \
    swarmssimparm.h

SOURCES += main.cpp

DISTFILES += \
    config/MapInfo.json \
    config/TaskInfo.json \
    config/USVPosInit.json
