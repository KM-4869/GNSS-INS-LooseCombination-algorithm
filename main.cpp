// IntegratedNavigation.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//


#include<fstream>
#include<iomanip>
#include<string>
#include"Matrix2.h"
#include"MyVector2.h"
#include"InertialNavigation.h"
#include"Kalman.h"
#include"LooseCombination.h"
#include"FileLoader.h"

#define STARTTIME 96111.0

using namespace std;

int main()
{

	//初始化载体位置速度姿态(单位：[m] [m/s] [deg])
	BLH Blh(30.5121294035, 114.3553132433, 25.643);
	Vector Vel(3, '|', -4.779, 17.747, 0.233);
	ANGLE Angle(109.00941143, -0.60798289, 1.73447908);
	NavState inistate(Blh, Vel, Angle);

	//初始化载体位置速度姿态的误差(单位：[m] [m/s] [deg])
	BLH Blh_std(0.010, 0.008, 0.013);
	Vector Vel_std(3, '|', 0.013, 0.010, 0.022);
	ANGLE Angle_std(0.05, 0.05, 0.05);
	NavState inistate_std(Blh_std, Vel_std, Angle_std);

	//GPS天线杆臂
	Vector al(3, '|', 0.0, 0.2380, 3.3650);
	//里程计杆臂
	Vector ol(3, '|', -0.05, -0.842, 0.5);

	//惯导传感器噪声参数
	Vector arw(3, 0.2);//输入参数单位为[deg/s/sqrt(h)]
	Vector vrw(3, 0.4);//输入参数单位为[m/s/sqrt(h)]
	Vector gb_std(3, 24.0);//输入参数单位为[deg/h]
	Vector ab_std(3, 400.0);//输入参数单位为[mGal]
	Vector gs_std(3, 1000.0);//输入参数单位为[ppm]
	Vector as_std(3, 1000.0);//输入参数单位为[ppm]
	
	IMUNoise ImuNoise(arw, vrw, gb_std, ab_std, gs_std, as_std, 3600.0);


	//构造运载体
	Body body(inistate, inistate_std, ImuNoise, al, ol);


	FileLoader fileloader_INS("D:\\组合导航\\2022组合导航课程考核与数据\\2022组合导航课程考核与数据\\L1.imd", 7, BINARY);
	FileLoader fileloader_GNSS("D:\\组合导航\\2022组合导航课程考核与数据\\2022组合导航课程考核与数据\\GNSS实验数据失锁2.txt", 13, TEXT);
	FileLoader fileloader_ODO("D:\\组合导航\\2022组合导航课程考核与数据\\2022组合导航课程考核与数据\\odo.bin", 2, BINARY);
	//fstream fout("D:\\组合导航\\2022组合导航课程考核与数据\\2022组合导航课程考核与数据\\组合导航解算数据NHCODO失锁.txt", ios::out);
	//fstream foutCov("D:\\组合导航\\2022组合导航课程考核与数据\\2022组合导航课程考核与数据\\组合导航解算数据方差分析2R.txt", ios::out);

	vector<double> OneLineImu(7,0.0);//一个历元的IMU数据
	vector<double> OneLineGNSS(13, 0.0);//一个历元的GNSS数据
	vector<double> OneLineODO(2, 0.0);//一个历元的ODO数据


	fileloader_GNSS.Skip_n_Line(5);//跳过5行数据，下一个读入数据为96112s

	while (OneLineODO[0] < STARTTIME)
	{
		fileloader_ODO.load(OneLineODO);
	}

	
	body.SetTimestamp(STARTTIME);

	while (!fileloader_INS.isEof())
	{
		fileloader_INS.load(OneLineImu);

		//跳过起始时刻之前的数据
		if (OneLineImu[0] < STARTTIME)
		{

			body.AddIMUData(OneLineImu.data());
			continue;
		}

		//添加IMU数据
		body.AddIMUData(OneLineImu.data());
		//添加ODO数据
		body.AddODOData(OneLineODO.data());
		fileloader_ODO.load(OneLineODO);
		
		//进行松组合算法传播误差状态和测量更新工作
		body.work();
		//body.BodyShow();
		//body.BodyOutputToFile(fout);
		//body.CovOutputToFile(foutCov);

		//若GNSS更新后读取一组新GNSS数据
		if (body.IsGNSSDataAvailable() == false)
		{
			fileloader_GNSS.load(OneLineGNSS);
			body.AddGNSSData(OneLineGNSS.data());
		}


	}
		
	//fout.close();
	//foutCov.close();

	//double t_INS = 0.0, t_1_INS = 96111.0;//时间与上一时刻时间
	//double t_GNSS = 0.0, t_1_GNSS = 96111.0;

	//Vector AngInc_k(3);//读取的k时刻角度增量向量
	//Vector AngInc_k_1(3);//k-1时刻角度增量向量
	//Vector VelInc_k(3);//读取的k时刻速度增量向量
	//Vector VelInc_k_1(3);//k-1时刻速度增量向量

	//Matrix F_k(21, 21);
	//Matrix F_k_1(21, 21);
	//Matrix G_k(21, 18);
	//Matrix G_k_1(21, 18);
	//Matrix H(6, 21);
	//Matrix A(21, 21);
	//Matrix q(18, 18);
	//q = CreateMatrixq(IMUError);
	//Matrix I_2121(21, 21);
	//I_2121.ToE(21);
	//Matrix q(21, 21);
	//Vector VP(21, '|', 0.010 * 0.010, 0.008 * 0.008, 0.013 * 0.013, 0.013 * 0.013, 0.010 * 0.010, 0.022 * 0.022, 0.00087266 * 0.00087266, 0.00087266 * 0.00087266, 0.00087266 * 0.00087266,
	//	0.00011635528 * 0.00011635528, 0.00011635528 * 0.00011635528, 0.00011635528 * 0.00011635528, 16.0, 16.0, 16.0,
	//	1.0E-6, 1.0E-6, 1.0E-6, 1.0E-6, 1.0E-6, 1.0E-6);
	//Matrix P(21, 21);
	//P = VP.diag();
	//Matrix x_predict(21, 1);
	//Matrix P_predict(21, 21);
	//

	

	//while ()
	//{
	//	fin_INS.read((char*)t_gyo_acc, 56);
	//	t_INS = t_gyo_acc[0];
	//	double delta_t_INS = t_INS - t_1_INS;

	//	if (t_gyo_acc[0] < 96110.999)
	//	{
	//		//AngInc_k_1.GetFormArray(1, t_gyo_acc);
	//		//VelInc_k_1.GetFormArray(4, t_gyo_acc);
	//		continue;
	//	}

	//	AngInc_k.GetFormArray(1, t_gyo_acc);
	//	VelInc_k.GetFormArray(4, t_gyo_acc);
	//	//内插出初始时刻0.002秒的增量
	//	if (abs(t_1_INS - 96111.0) < 1.0E-10)
	//	{
	//		//AngInc_k = 2 / 5 * AngInc_k;
	//		//VelInc_k = 2 / 5 * VelInc_k;
	//		
	//	}
	//	CreateMatrix_F_G_H(body, AngInc_k, VelInc_k, al, F_k_1, G_k_1, H, IMUError.T);

	//	//用k时刻GNSS观测值卡尔曼滤波估计惯导状态误差向量
	//	if (t_INS > t_GNSS)
	//	{
	//		double delta_t_GNSS = t_GNSS - t_1_GNSS;
	//		Matrix y(6, 1, '|', B_GNSS, L_GNSS, H_GNSS, VN_GNSS, VE_GNSS, VD_GNSS);

	//		CreateMatrix_F_G_H(body, AngInc_k, VelInc_k, al,  F_k, G_k, H,IMUError.T);

	//		A = I_2121 + F_k_1 * delta_t_GNSS;
	//		q = 1.0 / 2.0 * (A * G_k_1 * q * G_k_1.T() * A.T() + G_k * q * G_k.T()) * delta_t_GNSS;
	//		
	//		Matrix x(21, 1);
	//		Vector VR(6, '|', B_std_GNSS * B_std_GNSS, L_std_GNSS * L_std_GNSS, H_std_GNSS * H_std_GNSS, VN_std_GNSS * VN_std_GNSS, VE_std_GNSS * VE_std_GNSS, VD_std_GNSS * VD_std_GNSS);
	//		Matrix R(6, 6);
	//		R = VR.diag();
	//		kf_predict(x, P, A, q, x_predict, P_predict);
	//		kf_update(x_predict, P_predict, y, H, R, x, P);

	//		F_k_1 = F_k;
	//		G_k_1 = G_k;
	//		
	//		t_1_GNSS = t_GNSS;
	//		fin_GNSS >> t_GNSS >> B_GNSS >> L_GNSS >> H_GNSS >> B_std_GNSS >> L_std_GNSS >> H_std_GNSS >> VN_GNSS >> VE_GNSS >> VD_GNSS >> VN_std_GNSS >> VE_std_GNSS >> VD_std_GNSS;
	//	}
	//	//纯惯导机械编排解算
	//	body.Updating(AngInc_k_1, AngInc_k, VelInc_k_1, VelInc_k, t_INS - t_1_INS);
	//	AngInc_k_1 = AngInc_k;
	//	VelInc_k_1 = VelInc_k;
	//	t_1_INS = t_INS;

	//	cout << setprecision(6) << setiosflags(ios::fixed) << "t:" << t_INS << "  ";
	//	body.Show();

	//}

 //  



}

