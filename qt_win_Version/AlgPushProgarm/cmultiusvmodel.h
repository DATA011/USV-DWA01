#ifndef CMULTIUSVMODEL_H
#define CMULTIUSVMODEL_H

#include "cusvmodel.h"
#include "iomanip"

class cmultiUSVModel
{
public:
    cmultiUSVModel();

    //1.多艇初始化参数
    void multiInitialize(int flagModel);

    //2.多艇更新控制参数
    void updateMultiUSVControl(std::vector<USVInitStructType> multiUSVInitInfo, double curTime,int flagModel);

    //3.获取多艇的实时导航数据和探测到的目标
    void getMultiNaviObsInfo(int iterNum, std::vector<USVOutPutStructType> &multiUSVStatus, std::vector<InputOBSStruct> &curOBSFusion);

    //4.多艇探测障碍物融合
    std::vector<InputOBSStruct> multiDetectObsFUsionInfo(std::vector<InputOBSStruct> curOBS);

    //5、实时获取任务信息,当返回true时获取当前任务，当前任务被获取后，更新到下一任务。
    bool getTaskInfo(bool &isEnd, std::vector<SFormationTaskInfo> &taskPath, std::vector<referUSVInfo> &referInfo, int iterNum);

    //6、记录任务信息
    void RecordTaskInfo(const SFormationTaskInfo &referPointInfo, const std::vector<referUSVInfo> &referInfo, int taksID,int iterNum);

    //7、记录初始化信息
    void RecordInitInfo(std::vector<USVInitStructType> multiUSVInitInfo, int modelFlag);

    //8、任务结束，各艇的速度赋值为0
    void taskEnd(int iterNum,int modelFlag);

public:
    std::vector<SFormationTaskInfo> m_taskPath;   //编队航线
    std::vector<referUSVInfo> m_referInfo;    //编队构型
    std::vector<USVInitStructType> m_MultiUSVInitInfo;    //多无人艇初始信息


private:
    std::vector<USVInitStructType> m_updateMultiUSVInitInfo;    //多无人艇更新控制
    std::vector<USVOutPutStructType> m_multiUSVStatus;//多无人艇的导航信息
    std::vector<InputOBSStruct> m_multiDetectObsFUsion; //多艇探测融合障碍物
    SFormationTaskInfo lastTaskPoint; //上一个参考点

    std::vector<SFormationTaskInfo>       mySreferPointInfo;   //参考点信息
    std::vector<std::vector<referUSVInfo> >    mySreferInfo;        //编队构型


    CUSVModel *cmultiUSV;
    std::string str_timeTASK;
    std::string str_timeInit;
    bool TaskRecordFlag = false;    //任务信息记录标志
    int taskID = 1;

private:
    void updateTaskInfo(SFormationTaskInfo &referPointInfo, std::vector<referUSVInfo> &referInfo, int taskID);//更新任务

};

#endif // CMULTIUSVMODEL_H
