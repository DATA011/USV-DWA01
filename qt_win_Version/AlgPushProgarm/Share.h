#ifndef SHARED_H
#define SHARED_H

#include <string>
//using namespace std;

// 消息类型枚举类
enum {
	//自循环事件
	T_SelfWork=0
};


// 无人艇
struct  USVInitStructType
{
   double dbInitX;    //初始X坐标(米)
   double dbInitY;    //初始Y坐标(米)
   float fInitVel;      //初始速度(米/秒) 0~20 m/s
   float fInitYaw;      //初始航向(度)
   int iUSVType;        //无人艇类型，0-3T，1-7T，2-20T
   int iUSVId;          //无人艇ID
};

// 障碍物
struct  InputOBSStruct
{
   double dOBSX;    //x坐标
   double dOBSY;    //y坐标
   float fOBSV;     //速度(米/秒) 0~20 m/s
   float fOBSYaw;   //航向(度)
   double obsR;     //障碍物半径
   int    obsAtt;   //障碍物属性，1-已知障碍，2-探测障碍
};


//输入参数1：速度、航向
struct  USVInputStructVelYaw
{
   float fInputVel; //最新的速度(米/秒) 0~20 m/s
   float fInputYaw;//最新的航向(度))
   float fCurrTime; //切换航速航向控制时当前仿真时间
};

//输入参数2：下一航点
struct  USVInputStructPos
{
   double dbNextX;//下一航点X坐标(米)
   double dbNextY; //下一航点Y坐标(米)
   float fInputVel; // 0~20 m/s
   // float  fNextYaw; //下一个航迹点的的航向角度, 暂时不启用
   USVInputStructPos() {
      fInputVel = 0;
   }
};

//输出数据
struct  USVOutPutStructType
{
   double dbCurX; //实时的X坐标(米)
   double dbCurY; //实时的Y坐标(米)
   float fCurVel; //实时的速度(米/秒),
   float fCurYaw;//实时的航向(度))
   int iUSVId;          //无人艇ID
};

//记录数据
struct  USVRecordStructType
{
   double dbRecX; //记录的X坐标(米)
   double dbRecY; //记录的Y坐标(米)
   float fRecVel; //记录的速度(米/秒),
   float fRecYaw;//记录的航向(度))
};

// 潜艇
struct  SubmarineInitStructType
{
   double dbInitX; //初始X坐标(米)
   double dbInitY; //初始Y坐标(米)
   float fInitVel; //初始速度(米/秒),
   float fInitYaw;//初始航向(度)
   int iSubmarineType;  //潜艇类型,默认为0
};

//输入参数1：速度、航向
struct  SubmarineInputStructVelYaw
{
   float fInputVel; //最新的速度(米/秒),0~10m/s
   float fInputYaw;//最新的航向(度))
   float fCurrTime; //切换航速航向控制时候的时间
};

//输入参数2：下一航点
struct  SubmarineInputStructPos
{
   double dbNextX; //下一航点X
   double dbNextY; //下一航点Y
   float fInputVel;  // 航迹点控制时潜艇的速度 0-10m/s
   SubmarineInputStructPos(){
      fInputVel = 0;
   }

};

//输出数据
struct  SubmarineOutPutStructType
{
   double dbCurX; //实时的X,
   double dbCurY; //实时的Y,
   float fCurVel; //实时的速度(米/秒),
   float fCurYaw;//实时的航向(度))
};

//记录数据
struct  SubmarineRecordStructType
{
   double dbRecX; //记录的X,
   double dbRecY; //记录的Y,
   float fRecVel; //记录的速度(米/秒),
   float fRecYaw;//记录的航向(度))
};


//返回运行错误
struct  ErrOutType
{
   int     iObjectid;  //模型或对象ID
   std::string  sModelName; //模型名称
   int     iErrType; //错误类型： 1-一般错误，可继续运行；2-严重错误，停止运行
   std::string  sErrInfo; //用一段文字描述该错误，为便于查错定位，描述应尽量详细、具体
};


// grpc actions数据结构体，传向仿真实体
struct GrpcSpeedControl
{
   int eid;       //平台ID    
   std::string name;   //平台名称
   double bear;   //方向	 -1表示不控制,继承上次控制
   double sped;   //速度   -1表示不控制，继承上次控制
   float tm;      //在当前给定的航向上继续航行时间 0~4094秒，-1表示不控制，继承上次控制，默认为-1
};

struct GrpcPointControl
{
   int eid;  	      //平台ID  
   std::string name;   	//平台名称
   double X;       //X
   double Y;       //Y
   float sped; 	   //速度 m/s
};

//无人艇的obs数据
struct GrpcPlatsInfo
{
   //从模型GetOutPut获得的航速航向经度纬度，以及想定中给的信息
    double  Time ;								    //引擎实时时间
    std::string	PlatName ;				                //平台名称，从想定获取，目标名称
	int  ID ;						                //平台ID，从想定获取，实体编号
    int	Alliance;				                    //目标属性，从想定获取，指红蓝方中立方， 1-红方，2-蓝方，3-中立
    std::string  Medium;	                                //运动介质，从想定获取（平台类型），区分水面平台，水下平台,"SURFACE"-水面，"SUBSURFACE"-水下
	int	Type;		                                //平台类型 ，从想定获取(吨位)，0-3t艇，1-7t艇，2-20t艇，3-潜艇
	float	Speed;								    //速度  单位：米/秒，从模型获取，目标速度
	float	Bearing;							        //航向，从模型获取
    double	Y;								        //Y，从模型获取
    double	X;								        //X，从模型获取
    double	Alt;								        //高度，从想定获取，默认海面平台高度为0，水下平台高度为-200
    float	MaxSpeed;						        //平台最大速度，从想定获取，最大速度 单位：m/s
    int   optoelectronicDetectRange;             //无人艇载荷光电传感器，探测范围，从想定获取，单位：千米
    int   FoV ;                                   //无人艇载荷光电传感器，探测视场角，从想定获取，单位：度
    int   radarDetectRange;                      //无人艇载荷导航雷达，探测范围，从想定获取，单位：千米
    int   radarWidth ;                            //无人艇载荷导航雷达，探测视场角，从想定获取，单位：度
    int   lidarDetectRange ;                      //无人艇载荷激光雷达，探测范围，从想定获取，单位:千米
    int   lidarWidth ;                            //无人艇载荷激光雷达，探测视场角，从想定获取，单位：度
    int   sonarDetectRange ;                      //无人艇载荷声纳，探测范围，从想定获取，单位：千米
    int   sonarWidth ;                            //无人艇载荷声纳，探测视场角，从想定获取，单位：度
    int   weaponAttackRange ;                     //无人艇载荷武器，攻击范围，从想定获取，单位：千米

   void Copy(GrpcPlatsInfo & data)
   {
      data.Time = Time;
      data.PlatName = PlatName;
      data.ID = ID;
      data.Alliance = Alliance;
      data.Medium = Medium;
      data.Type = Type;
      data.Speed = Speed;
      data.Bearing = Bearing;
      data.Y = Y;
      data.X = X;
      data.Alt = Alt;
      data.MaxSpeed = MaxSpeed;
      data.optoelectronicDetectRange = optoelectronicDetectRange;
      data.FoV = FoV;
      data.radarDetectRange = radarDetectRange;
      data.radarWidth = radarWidth;
      data.lidarDetectRange = lidarDetectRange;
      data.lidarWidth = lidarWidth;
      data.sonarDetectRange = sonarDetectRange;
      data.sonarWidth = sonarWidth;
      data.weaponAttackRange = weaponAttackRange;
   }
};

// 潜艇的obs数据
struct GrpcSubInfo {
    double  Time ;								    //引擎实时时间
    std::string	PlatName ;			                //平台名称，从想定获取，目标名称
    int   ID ;						                //平台ID，从想定获取，实体编号
    int	Alliance;			                    //目标属性，从想定获取，指红蓝方中立方， 1-红方，2-蓝方，3-中立
    std::string  	Medium;                                //运动介质，从想定获取（平台类型），区分水面平台，水下平台,"SURFACE"-水面，"SUBSURFACE"-水下
    int	Type;	                                //平台类型 ，从想定获取(吨位)，0-3t艇，1-7t艇，2-20t艇，3-潜艇
    float	Speed;							    //速度  单位：米/秒，从模型获取，目标速度
    float	Bearing;						        //航向，从模型获取
    double	Y;							        //Y，从模型获取
    double	X;								        //X，从模型获取
    double	Alt;								        //高度，从想定获取，默认海面平台高度为0，水下平台高度为-200
    float	MaxSpeed;						        //平台最大速度，从想定获取，最大速度 单位：m/s

   void Copy(GrpcSubInfo & data)
   {
      data.Time = Time;
      data.PlatName = PlatName;
      data.ID = ID;
      data.Alliance = Alliance;
      data.Medium = Medium;
      data.Type = Type;
      data.Speed = Speed;
      data.Bearing = Bearing;
      data.Y = Y;
      data.X = X;
      data.Alt = Alt;
      data.MaxSpeed = MaxSpeed;
   }
};

// 浮标的obs数据
struct GrpcBuoyInfo {
    std::string BuoyName;                                //浮标名称，从想定获取，目标名称
    int ID;                                       //浮标ID，从想定获取，实体编号
    //int32 entityType = 3;//实体类型，1-3吨无人艇，2-7吨无人艇，3-20吨无人艇，4-潜艇，5-浮标
    double Y;                                     //Y，从想定获取，单位：度
    double X;                                     //X，从想定获取，单位：度
    double Alt;                                     //高度，从想定获取，单位：米

    void Copy(GrpcBuoyInfo & data)
    {
      data.BuoyName = BuoyName;
      data.ID = ID;
      data.X = X;
      data.Y = Y;
      data.Alt = Alt;     
    }
};

/*参考点*/
struct SFormationTaskInfo
{
    double      referX;                 //参考点X
    double      referY;                 //参考点Y
    double      referR;                 //参考点半径
    double      referSpeedMin;          //参考速度最小值
    double      referSpeedMax;          //参考速度最大值
    double      referYaw;               //参考航向  与正北航向顺时针递增，范围【0，360】

};

/*参考艇信息*/
struct referUSVInfo
{
    int             type;           //类型
    int             USVID;          //无人艇编号     000标识不约束
    double          relatDis;       //相对距离       WRT相对参考WRT的距离，单位米
    double          relatYaw;       //相对角度       WRT相对参考WRT的角度，单位度（以参考艇为圆心，正北顺时针旋转的角度）
};


#endif // SHARED_H
