// 2015 7.15  Noitom xyz
// 速度计算参数设置

#pragma once

class CVelocityCalPS
{
public:
	CVelocityCalPS();
	~CVelocityCalPS();

	float dT_VnsV;		// 计算视觉 速度 用的时间步长 sec
	float dT_VnsA;		// 计算视觉 加速度 用的时间步长 sec
	float dT_VnsA_V;	// 计算视觉 加速度的速度 用的时间步长 sec

	float dT_InsA_V;	// 计算惯性 加速度的速度 用的时间步长 sec

	int VCalMethod;
	
private:

};

