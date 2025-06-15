#ifndef CUSVMODEL_H
#define CUSVMODEL_H

#include <string>
#include <vector>
#include <iostream>
#include "Share.h"
#include "CUSVCommon.h"
#include "swarmssimparm.h"

#include<fstream>
#include<iomanip>
#include<unistd.h>
#include<windows.h>


using namespace CUSV;
//9、数字加密
double digitalEncry(double numValues);

//10、数字解密
double UdigitalEncry(double numValues);

class  CUSVModel
{

public:
    //服务接口
    //1.设置初始化参数, 参数1:初始化数据，参数2:运行错误信息，参数3：是否打印调试输出信息
    int Initialize(USVInitStructType *sUSVInitData, std::vector<ErrOutType *>*pErrInfo, bool DebugInfoOutput);

    //2.输入参数设置, 参数1:输入参数，参数2:运行错误信息
    //设置最新的航速(米/秒)、航向(度)
    int SetInput(USVInputStructVelYaw *sVelYaw, std::vector<ErrOutType *>*pErrInfo);

    //设置下一航路点
    int SetInput(USVInputStructPos *sPos, std::vector<ErrOutType *>*pErrInfo);

    //3.仿真模型解算, 参数1:仿真时间(单位，秒)，参数2:入口函数名称，参数3:运行错误信息
    void ModelProcess(double dbSimTime);

    //4.得到模型输出数据, 得到最新解算USV的位置、速度、航向，参数1:模型输出数据
    int GetOutput(USVOutPutStructType *sUSVStatus);

    //5.得到记录数据, 得到需记录的USV的位置、速度、航向，参数1:记录数据
    void RecordUSVInfo(const USVOutPutStructType &sUSVStatus,int iterNum);

    //6、当无人艇与障碍物的安全边界距离小于R时，记录信息，包含仿真时刻，无人艇id，无人艇位置（x，y），障碍物序号id，障碍物半径
    void RecordObsAvoidInfo(const USVOutPutStructType &sUSVStatus, std::vector<InputOBSStruct> &curOBS,int iterNum,double detectR);

    //7、获取采样时间
    float getDeltatTime();

    //8、获取所有艇的控制信息，判断那些是自己执行的
    void setMultiUSVControlInfo(std::vector<USVInitStructType> multiUSVInitInfo, double curTime, int flagModel);

    CUSVModel();

private://模型自定义函数、变量
    std::string str_timeUSV;   //导航记录当前时间
    std::string str_timeOBS;   //导航记录当前时间


    bool USVNaviRecordFlag;//
    bool OBSRecordFlag;//
    int USVNaviRecordNum;//
    int OBSRecordNum;//
    int usvID;
    double dbInitX; //初始X
    double dbInitY; //初始Y
    double dbX; //当前X
    double dbY; //当前Y
    double dbNewX;//新设置的航路点X
    double dbNewY;//新设置的航路点Y
    double q0[3] = {0.0, 0.0, 0.0}; //初始平面坐标（x,y,yaw）
    double *q1 ;   // 下一个航迹点的坐标（x,y,yaw）
    double q[4] = {0.0, 0.0, 0.0, 0.0}; // 当前无人艇的状态 [x, y, yaw， v]
    double dubins_q[3] = {0.0, 0.0, 0.0}; //(x,y,yaw)
    float fNewVel; // 新设置的速度
    float fNewYaw; // 新设置的航向
    float fVel; //当前的速度(米/秒)
    float fYaw;//当前的航向(度))
    int iUSVType;  //无人艇类型，1-3T，2-7T，3-50T
    int iCtlMethod  = 3;//0 航路点控制模式, 1:航速航向控制模式
    CUSV::DubinsPath path; // dubins 结构体
    double turining_radius; // 无人艇转弯半径
    double distance;      // 记录已经走过的距离
    double length;        // 路径规划得到的路径距离
    bool bDebugInfoOutput;//调试输出模式，false:不输出，true:输出
    float deltat = 0.1;  //采样时间
    float fCurrTime; //当前时间
    CUSV::PID_param yaw_pid;

    /***************** function ****************/
    /**
     * @param mypid: pid相关参数
     * @param now: 当前状态
     * @param goal： 目标状态
     *
     * @return 输出控制变量，例如控制航向角，输出为角速度
    */
    float PID_update(CUSV::PID_param *mypid, float now, float goal);

    void USV_path_generator(float yaw_des, float v_des, float t, float deltat, float fCurrTime, const double* q0 ,double q[4], PID_param* yaw_pid);

};

#endif // CUSVMODEL_H
