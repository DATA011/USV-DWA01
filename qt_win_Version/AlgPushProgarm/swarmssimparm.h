#ifndef SWARMSSIMPARM_H
#define SWARMSSIMPARM_H


#include <QDebug>
#include <QtCore>
#include <QFile>
#include <QXmlStreamReader>
#include <iostream>
#include <QMutex>
#include<fstream>
#include<iomanip>
#include<unistd.h>

#include "Share.h"
#include "cusvmodel.h"

class SwarmsSimParm
{
public:
    SwarmsSimParm();
    static SwarmsSimParm* getInstance();

    //Json读取
    QJsonObject loadCommonConfig(const QString &filename)const ;

    /*************************************************************************
      *
      * 函数名称 : readInfoJason
      * 函数描述 : 读取初始无人艇位置信息
      * 返回类型 :
      * 参数说明 :
      */
    void readUSVPosInitInfoJason(const QString &filenamePath);

    /*************************************************************************
      *
      * 函数名称 : readInfoJason
      * 函数描述 : 读取障碍信息信息
      * 返回类型 :
      * 参数说明 :
      */
    void readMapObsInfoJason(const QString &filenamePath);

    /*************************************************************************
      *
      * 函数名称 : readTaskInfoJason
      * 函数描述 : 读取任务信息
      * 返回类型 :
      * 参数说明 :
      */
    void readTaskInfoJason(const QString &filenamePath);


    //jason
     std::vector<USVInitStructType> m_USVInitInfo;    //无人艇初始信息
     std::vector<InputOBSStruct> m_OBSInfo;    //障碍物信息
     std::vector<SFormationTaskInfo>       SreferPointInfo; //参考点信息
     std::vector<std::vector<referUSVInfo> >    SreferInfo;     //编队构型
     double      referStartX;            //起始点参考点X
     double      referStartY;            //起始点参考点Y


private:
    static SwarmsSimParm*  m_pInstance;

};

#endif // SWARMSPARM_H
