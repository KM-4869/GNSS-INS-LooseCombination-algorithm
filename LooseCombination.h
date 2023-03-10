#pragma once

#include"InertialNavigation.h"
#include"Constant.h"
#include<iomanip>//设置输出格式
#include<fstream>
using namespace std;
struct NavState
{
	BLH Blh;
	Vector Vel;
	ANGLE Angle;

	NavState(BLH B,Vector V,ANGLE A):Vel(3)
	{
		Blh = B;
		Vel = V;
		Angle = A;
	}

};

struct GNSSDATA
{
	double time;

	Vector Blh;
	Vector Vel;

	Vector Blh_std;
	Vector Vel_std;

	bool DataAvailable;

	GNSSDATA():Blh(3),Vel(3),Blh_std(3),Vel_std(3)
	{
		time = 0.0;
		DataAvailable = false;
	}
};



class Body :public IMUBOX
{

public:

	Body(NavState inistate, NavState inistate_std, IMUNoise imun, Vector al, Vector ol);

	void AddIMUData(double* OneLineIMU);
	void AddGNSSData(double* OneLineGNSS);
	void AddODOData(double* OneLineODO);

	void SetTimestamp(double time);
	bool IsGNSSDataAvailable();

	void work();
	

	void BodyShow();
	void BodyOutputToFile(fstream& fout);
	void CovOutputToFile(fstream& fout);

private:

	IMUDATA IMUData_pre, IMUData_cur;//前一时刻和现在的IMU数据
	GNSSDATA GNSSData;//GNSS数据

	double timestamp;//每次要进行内插的时间记号（通常为初始时刻和有GNSS数据整秒）

	IMUError ImuError;
	IMUNoise ImuNoise;

	Vector antlever;//杆臂（由b->GNSS,在b系下测量）
	Vector odolever;//里程计杆臂

	Matrix Cov;//状态协方差（21*21）
	Matrix q;//系统噪声矩阵（18*18）
	Matrix x;//待估的参数（21*1）

	double odoVel;


	void ImuInterpolate(IMUDATA& IMUData_pre, IMUDATA& IMUData_cur, double midtime, IMUDATA& IMUData_mid);//IMU内插
	
	void ImuPropagation(IMUDATA& IMUData_pre, IMUDATA& IMUData_cur);//IMU状态传播
	void GNSSUpdating(IMUDATA& IMUData_mid);//GNSS测量更新

	void Feedback();//状态反馈
	void Compensate();//IMU数据误差补偿

	bool IsVelocityZero();
	void ZUPT();//零速测量更新

	//bool IsNHC();
	void NHC();//侧向垂向速度为0运动约束测量更新

	void ODO();//由里程计提供的前向速度的测量更新

	//void NHC_ODO();



};


