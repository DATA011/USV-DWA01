#include "cusvmodel.h"
#include "cmultiusvmodel.h"
#include <vector>
#include <QDebug>
#include <cmath>
#include <algorithm>

using namespace std;

//虚拟艇
float v_x = 500; //800
float v_y = 0;
float vir_Vel = 8;

// float T = 400;            //仿真总时间
float dt = 0.1;           //仿真时间步长
// float steps = T / dt;     // 仿真步数

float v_min = 3;
float v_max = 15;
float dv_max = 3;       //最大速度变化率0.3
float w_max = M_PI/3;       // 最大角速度
float r_safe = 50;          // 船间最小安全距离60
float predict_time = 15;    //DWA预测窗口时间30
float predict_dt = 0.5;     //DWA预测单位时间0.5
float max_steps = predict_time/predict_dt;
float w_cur = 0;
float score_penalty = 0;
float total_score = 0;
float w;
float v;
int num;
bool safe = true;
float v_opt,w_opt;
int vk;
int vp;
int bb = 0;
int f,ff;
std::vector<InputOBSStruct> curOBSFusion; //态势信息

struct s1{
    float rex;
    float rey;
    float id;
};
s1 temp;
vector <s1> v1;

struct IndexDistance {
    int index;
    float distance;
};
vector<IndexDistance> v2;

struct s2{
    float rx;
    float ry;
};
vector <s2> v3;

struct s3{
    float rx1;
    float ry1;
};
vector <s3> v4;
vector <s3> v5;

bool compareIndexDistance(const IndexDistance& a, const IndexDistance& b) {
    return a.distance < b.distance;
}
bool compareIndexDistance1(const s1& a, const s1& b) {
    return a.id < b.id;
}

void init(const vector<referUSVInfo>& referInfo){
    for(int i = 0;i<referInfo.size();i++){
        if(referInfo[i].USVID == 0){
            if(referInfo[i].type==1){
                temp.rex = -50;
                temp.rey = 0;
                temp.id = 0;
                // qDebug()<<"rex = "<< temp.rex << "rey = "<<temp.rey;
                v1.push_back(temp);
            }else{
                float x1 = referInfo[i].relatDis * cos((90-referInfo[i].relatYaw)*M_PI/180);
                float y1 = referInfo[i].relatDis * sin((90-referInfo[i].relatYaw)*M_PI/180);
                temp.rex = x1 - 50;
                temp.rey = y1;
                temp.id = 0;
                // qDebug()<<"rex = "<< temp.rex << "rey = "<<temp.rey;
                v1.push_back(temp);
            }
        }else{
            if(referInfo[i].type==1){
                temp.rex = -50;
                temp.rey = 0;
                temp.id = referInfo[i].USVID;
                // qDebug()<<"rex = "<< temp.rex << "rey = "<<temp.rey;
                v1.push_back(temp);
            }else{
                float x1 = referInfo[i].relatDis * cos((90-referInfo[i].relatYaw)*M_PI/180);
                float y1 = referInfo[i].relatDis * sin((90-referInfo[i].relatYaw)*M_PI/180);
                temp.rex = x1 - 50;
                temp.rey = y1;
                temp.id = referInfo[i].USVID;
                // qDebug()<<"rex = "<< temp.rex << "rey = "<<temp.rey;
                v1.push_back(temp);
            }
        }
    }
    if(v1[0].id != 0){
        std::sort(v1.begin(), v1.end(), compareIndexDistance1);
    }

    // for(int i = 0;i<v1.size();i++){
    //     qDebug()<<v1[i].id<<" "<<v1[i].rex<<" "<<v1[i].rey;
    // }
}
void predict_trajectory(int i,float w,float v,float max_steps,const vector<USVOutPutStructType> multiUSVStatus){
    float theta = (90-multiUSVStatus[i].fCurYaw)*M_PI/180;
    float cx = multiUSVStatus[i].dbCurX;
    float cy = multiUSVStatus[i].dbCurY;
    // qDebug()<<"theta"<<theta;
    for(int j = 0;j<max_steps;j++){
        cx = cx + v *cos(theta)*predict_dt;
        cy = cy + v *sin(theta)*predict_dt;
        theta = theta + w * predict_dt;
        s3 tok1;
        tok1.rx1 = cx;
        tok1.ry1 = cy;
        v4.push_back(tok1);
    }
}
void predict_trajectory1(int i,float w,float v,float max_steps,const vector<USVOutPutStructType> multiUSVStatus){
    v5.clear();
    float theta = (90-multiUSVStatus[i].fCurYaw)*M_PI/180;
    float cx = multiUSVStatus[i].dbCurX;
    float cy = multiUSVStatus[i].dbCurY;
    for(int j = 0;j<max_steps;j++){
        cx = cx + v *cos(theta)*predict_dt;
        cy = cy + v *sin(theta)*predict_dt;
        theta = theta + w * predict_dt;
        s3 tok2;
        tok2.rx1 = cx;
        tok2.ry1 = cy;
        v5.push_back(tok2);
    }
}
void is_safe_trajectory(int i,vector <s3> vv,const vector<USVOutPutStructType> multiUSVStatus){
    safe = true;
    bool should_break = false;
    for(int k = 0;k < num;k++){
        if(v2[k].index == i){
            vk = k;
        }
    }
    for(int h = 0;h<max_steps;h++){
        //与障碍物碰撞检测
        // for(int d = 0;d < curOBSFusion.size();d++){
        //     if(sqrt((vv[h].rx1-curOBSFusion[d].dOBSX)*(vv[h].rx1-curOBSFusion[d].dOBSX)+
        //              (vv[h].ry1-curOBSFusion[d].dOBSY)*(vv[h].ry1-curOBSFusion[d].dOBSY))<r_safe+curOBSFusion[d].obsR){
        //         safe = false;
        //         should_break = true;
        //         break;
        //     }else{
        //         safe = true;
        //     }
        // }
        // if(should_break == true){
        //     break;
        // }

        //与其他船之间
        for(int j = 0;j<num;j++){
            if(j == i){
                continue;
            }else{
                if(sqrt((vv[h].rx1-multiUSVStatus[j].dbCurX)*(vv[h].rx1-multiUSVStatus[j].dbCurX)+
                         (vv[h].ry1-multiUSVStatus[j].dbCurY)*(vv[h].ry1-multiUSVStatus[j].dbCurY))<r_safe){
                    safe = false;
                    should_break = true;
                    break;
                }else{
                    safe = true;
                }
            }
        }
        if(should_break == true){
            break;
        }
    }
    // for(int h = 0;h<max_steps;h++){
    //     //与障碍物碰撞检测
    //     // for(int d = 0;d < curOBSFusion.size();d++){
    //     //     if(sqrt((vv[h].rx1-curOBSFusion[d].dOBSX)*(vv[h].rx1-curOBSFusion[d].dOBSX)+
    //     //              (vv[h].ry1-curOBSFusion[d].dOBSY)*(vv[h].ry1-curOBSFusion[d].dOBSY))<r_safe+curOBSFusion[d].obsR){
    //     //         safe = false;
    //     //         should_break = true;
    //     //         break;
    //     //     }else{
    //     //         safe = true;
    //     //     }
    //     // }
    //     // if(should_break == true){
    //     //     break;
    //     // }

    //     //与其他船之间
    //     for(int j = 0;j<num;j++){
    //         if(j == i){
    //             continue;
    //         }else{
    //             for(int k = 0;k < num;k++){
    //                 if(v2[k].index == j){
    //                     vp = k;
    //                 }
    //             }
    //             if(vk>vp){
    //                 if(sqrt((vv[h].rx1-multiUSVStatus[j].dbCurX)*(vv[h].rx1-multiUSVStatus[j].dbCurX)+
    //                          (vv[h].ry1-multiUSVStatus[j].dbCurY)*(vv[h].ry1-multiUSVStatus[j].dbCurY))<r_safe){
    //                     safe = false;
    //                     should_break = true;
    //                     break;
    //                 }else{
    //                     safe = true;
    //                 }
    //             }
    //         }
    //     }
    //     if(should_break == true){
    //         break;
    //     }
    // }
}

void dwa_controller_priority(int i,const vector<USVOutPutStructType> multiUSVStatus){
    float best_score = -std::numeric_limits<float>::infinity();
    w = 0;
    v = multiUSVStatus[i].fCurVel;
    bool can_break = false;
    predict_trajectory(i,w,v,max_steps,multiUSVStatus);
    is_safe_trajectory(i,v4,multiUSVStatus);
    if(safe == true){
        //使用 PID 控制（无碰撞风险时）
        float Kp_ang = 2; //2.0
        float Kp_v = 0.1; //0.1
        float ddx = v3[0].rx - multiUSVStatus[i].dbCurX;
        float ddy = v3[0].ry - multiUSVStatus[i].dbCurY;
        float desired_theta = atan2(ddy, ddx);
        float error_theta = atan2(sin(desired_theta - (90-multiUSVStatus[i].fCurYaw)*M_PI/180),
                                  cos(desired_theta - (90-multiUSVStatus[i].fCurYaw)*M_PI/180));
        float dist_to_goal = sqrt(ddx*ddx+ddy*ddy);
        v_opt = Kp_v*dist_to_goal;
        if(v_opt < v_min){
            v_opt = v_min;
        }
        if(v_opt > v_max){
            v_opt = v_max;
        }
        w_opt = Kp_ang * error_theta;
        can_break = true;
    }
    if(can_break == false){
        //使用 DWA 控制器
        // qDebug()<<"使用了DWA控制器！";
        bool can_find = false;
        float v_start = std::max(v_min, multiUSVStatus[i].fCurVel - dv_max);
        float v_end = std::min(v_max, multiUSVStatus[i].fCurVel + dv_max);
        for (v = v_start; v <= v_end; v += 0.1) {
            for (w = -M_PI/3; w <= M_PI/3; w += M_PI/18) {
                predict_trajectory1(i,w,v,max_steps,multiUSVStatus);
                s3 lastElement = v5.back();
                float dist_to_goal = sqrt((lastElement.rx1 - v3[0].rx)*(lastElement.rx1 - v3[0].rx)+(lastElement.ry1 - v3[0].ry)*(lastElement.ry1 - v3[0].ry));
                float heading_score = -dist_to_goal;
                is_safe_trajectory(i,v5,multiUSVStatus);
                if(safe == false){
                    continue;
                }
                can_find = true;
                for(int k = 0;k < num;k++){
                    if(v2[k].index == i){
                        vk = k;
                    }
                }
                score_penalty = 0;
                for(int j = 0;j < num;j++){
                    for(int k = 0;k < num;k++){
                        if(v2[k].index == j){
                            vp = k;
                        }
                    }
                    if(vk>vp){
                        if(j != i){
                            float fg = sqrt((lastElement.rx1-multiUSVStatus[j].dbCurX)*(lastElement.rx1-multiUSVStatus[j].dbCurX)+
                                            (lastElement.ry1-multiUSVStatus[j].dbCurY)*(lastElement.ry1-multiUSVStatus[j].dbCurY));
                            if(fg < r_safe * 2){
                                score_penalty = score_penalty - 100;
                            }
                        }
                    }
                }
                total_score = heading_score + score_penalty;
                if(total_score > best_score){
                    best_score = total_score;
                    // qDebug()<<"best_score"<<best_score;
                    v_opt = v;
                    w_opt = w;
                }
            }
        }
        if(can_find == false){
            v_opt = (multiUSVStatus[i].fCurVel)/5;
            if(v_opt < 3){
                v_opt = 3;
            }
            w_opt = 0;
        }
    }
}

//算法实现
std::vector<USVInitStructType> algorithmImpl(int algIterNum,std::vector<SFormationTaskInfo> taskPath,std::vector<referUSVInfo> referInfo,std::vector<USVOutPutStructType> multiUSVStatus)
{

    v_x = v_x + vir_Vel * dt;//虚拟艇速度为8

    // qDebug()<<curOBSFusion.size();
    // for(int d = 0;d < curOBSFusion.size();d++){
    //     qDebug()<<curOBSFusion[d].dOBSX<<" "<<curOBSFusion[d].dOBSY<<" "<<curOBSFusion[d].obsR;
    // }

    // for(int i = 0;i<multiUSVStatus.size();i++){
    //     qDebug()<<multiUSVStatus[i].iUSVId;
    // }

    num = multiUSVStatus.size();
    float distance = 0;

    // for (int i = 0; i < multiUSVStatus.size(); i++) {
    //     float distance = sqrt((v_x - multiUSVStatus[i].dbCurX) * (v_x - multiUSVStatus[i].dbCurX) + (v_y - multiUSVStatus[i].dbCurY) * (v_y - multiUSVStatus[i].dbCurY));
    //     v2.push_back({i, distance});
    // }

    for (int i = 0; i < multiUSVStatus.size(); i++) {
        if(multiUSVStatus[i].dbCurX > v_x){
            distance = -sqrt((v_x - multiUSVStatus[i].dbCurX) * (v_x - multiUSVStatus[i].dbCurX) + (v_y - multiUSVStatus[i].dbCurY) * (v_y - multiUSVStatus[i].dbCurY));
        }else{
            distance = sqrt((v_x - multiUSVStatus[i].dbCurX) * (v_x - multiUSVStatus[i].dbCurX) + (v_y - multiUSVStatus[i].dbCurY) * (v_y - multiUSVStatus[i].dbCurY));
        }
        v2.push_back({i, distance});
    }

    // for (int i = 0; i < multiUSVStatus.size(); i++) {
    //     distance = multiUSVStatus[i].dbCurX;
    //     v2.push_back({i, distance});
    // }

    // for(int i = 0; i < v2.size(); i++){
    //     qDebug()<<v2[i].index<<v2[i].distance;
    // }

    std::sort(v2.begin(), v2.end(), compareIndexDistance);

    // qDebug()<<"------------------------";

    // for(int i = 0; i < v2.size(); i++){
    //     qDebug()<<v2[i].index<<v2[i].distance;
    // }

    for (int i = 0; i < multiUSVStatus.size(); i++) {
        s2 tok;
        tok.rx = v1[i].rex + v_x;
        tok.ry = v1[i].rey + v_y;
        v3.push_back(tok);
        // qDebug()<<v3[i].rx<<" "<<v3[i].ry;
        dwa_controller_priority(i,multiUSVStatus);
        // qDebug()<<"safe:"<<safe;
        // float gh = std::max(-dv_max, std::min(, v_opt - multiUSVStatus[i].fCurVel));
        // multiUSVStatus[i].fCurVel = multiUSVStatus[i].fCurVel + gh;
        multiUSVStatus[i].fCurVel = v_opt;
        multiUSVStatus[i].fCurYaw = 90-(90-multiUSVStatus[i].fCurYaw + w_opt * dt * 180/M_PI);
        v3.clear();
        v4.clear();
        v5.clear();
    }
    v2.clear();

    for(int m = 0;m<multiUSVStatus.size();++m){
        for(int j = 0;j<multiUSVStatus.size();++j)
        {
            if(j == m)
            {
                continue;
            }
            else
            {
                float s = sqrt(pow((multiUSVStatus[m].dbCurX - multiUSVStatus[j].dbCurX),2) +  pow((multiUSVStatus[m].dbCurY - multiUSVStatus[j].dbCurY),2));
                if(s<=50)
                {
                    f = 1;
                }
            }
        }
    }

    for(int m = 0;m<multiUSVStatus.size();++m){
        for(int j = 0;j<curOBSFusion.size();++j)
        {
            float s = sqrt(pow((multiUSVStatus[m].dbCurX - curOBSFusion[j].dOBSX),2) +  pow((multiUSVStatus[m].dbCurY - curOBSFusion[j].dOBSY),2));
            if(s<=50+curOBSFusion[j].obsR)
            {
                ff = 1;
            }
        }
    }

    //根据实时位置，任务信息求下一时刻艇的期望航速航向
    //    std::vector<SFormationTaskInfo> taskPath;   //编队航线
    //    std::vector<referUSVInfo> referInfo;    //编队构型
    //    std::vector<USVOutPutStructType> multiUSVStatus;  //编队内艇信息
    std::vector<USVInitStructType> multiUSVInitInfo;
    for(int i = 0;i<multiUSVStatus.size();++i)
    {
        USVInitStructType updateUSVControl;
        updateUSVControl.iUSVId = multiUSVStatus[i].iUSVId;
        updateUSVControl.dbInitX = multiUSVStatus[i].dbCurX;
        updateUSVControl.dbInitY = multiUSVStatus[i].dbCurY;
        updateUSVControl.fInitVel = multiUSVStatus[i].fCurVel;//multiUSVStatus[i].fCurVel;    //速度(米/秒),
        updateUSVControl.fInitYaw = multiUSVStatus[i].fCurYaw;//multiUSVStatus[i].fCurYaw;    //航向(度))
        multiUSVInitInfo.push_back(updateUSVControl);
    }
    return multiUSVInitInfo;
}

void initCSVFile(const std::string& filename) {
    std::ofstream file(filename);
    file << "Step,id,x,y,Vel,Yaw\n";
    file.close();
}


void logMultiUSVStatus(const std::vector<USVOutPutStructType>& multiUSVStatus, const std::string& filename, int Step) {
    std::ofstream file(filename, std::ios::app);
    if (file.is_open()) {
        for (const auto& usv : multiUSVStatus) {
            file << Step << "," << usv.iUSVId << "," << usv.dbCurX << "," << usv.dbCurY << "," << usv.fCurVel<< "," << usv.fCurYaw << "\n";
        }
        file << Step << "," << multiUSVStatus.size()+1 << "," << v_x << "," << v_y << "," << vir_Vel<< "," << 90 << "\n";
        file.close();
    }
}

int main()
{
    cmultiUSVModel cmultilUSV;
    std::vector<SFormationTaskInfo>  taskPath; //任务航路
    std::vector<referUSVInfo> referInfo; //构型信息
    std::vector<USVOutPutStructType> multiUSVStatus; //导航信息
    // std::vector<InputOBSStruct> curOBSFusion; //态势信息
    std::vector<USVInitStructType> multiUSVInitInfo;  //各艇控制指令
    int flagModel = 1; //控制模式，1-航速航向；2-航路，建议使用航速航向
    int iterNum = 0; //整个任务迭代次数
    int algIterNum = 0; //算法更新后清零
    bool rcvFlag;   //任务触发标志
    cmultilUSV.multiInitialize(flagModel);  //初始化

    std::string filename = "multi_usv_log.csv";
    initCSVFile(filename);

    while(1)
    {
        //更新控制指令
        cmultilUSV.updateMultiUSVControl(multiUSVInitInfo,iterNum,flagModel);
        //获取导航信息和态势信息
        cmultilUSV.getMultiNaviObsInfo(iterNum,multiUSVStatus,curOBSFusion);
        //获取任务信息
        bool isEnd;
        rcvFlag = cmultilUSV.getTaskInfo(isEnd,taskPath,referInfo,iterNum);
        //触发任务
        if(true == rcvFlag)
        {
            if(true == isEnd)//到达最后一个任务点，不再发送任务
            {
                cmultilUSV.taskEnd(iterNum,flagModel);
                break;  //跳出算法循环
            }
            algIterNum = 0; //新任务，次数清零
        }
        if(bb == 0){
            init(referInfo);
            bb++;
        }
        // if(f == 1){
        //     qDebug()<<"BOAT BUMP!!!!!";
        //     // break;
        // }
        // if(ff == 1){
        //     qDebug()<<"OBS BUMP!!!!!";
        // }
        //算法更新控制指令
        multiUSVInitInfo = algorithmImpl(algIterNum,taskPath,referInfo,multiUSVStatus);
        // break;

        logMultiUSVStatus(multiUSVStatus,filename,iterNum);
        //迭代次数更新
        ++iterNum;
        ++algIterNum;
    }
    if(f == 1){
        qDebug()<<"BOAT BUMP!!!!!";
        // break;
    }
    if(ff == 1){
        qDebug()<<"OBS BUMP!!!!!";
    }
}

