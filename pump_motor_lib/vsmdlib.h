#pragma once


#ifdef VSMD_DLL_EXPORTS
#define VSMD_DLL_API __declspec(dllexport)
#else
#define VSMD_DLL_API __declspec(dllimport)
#endif


class VSMD_DLL_API VsmdDevice
{
protected:

public:
	VsmdDevice(int cid);
	virtual ~VsmdDevice();

	// 打开后台自动更新状态功能（获取速度，位置，状态位信息）
	virtual void autoStsOn();
	// 关闭后台自动更新状态功能（获取速度，位置，状态位信息）
	virtual void autoStsOff();

	// 电机使能
	virtual void enable();
	// 电机失能
	virtual void disable();
	// 速度模式连续转动
	virtual void move();
	// 绝对位置移动
	// pos - 绝对位置
	virtual void moveto(int pos);
	// 相对位置移动
	// distance - 移动距离
	virtual void rmove(int32_t distance);
	// 停止
	// mode=0 - 减速停止
	// mode=1 - 立刻停止
	virtual void stop(int mode);
	// 发送获取状态指令（可不用）
	virtual void sts();
	// 发送获取驱动器版本号指令（可不用）
	virtual void dev();
	// 设置当前位置为0点
	virtual void org();
	// 保存参数到驱动器（程序运行时不可使用）
	virtual void save();
	// 设置速度
	virtual void cfgSpd(float spd);
	// 设置加速度
	virtual void cfgAcc(float acc);
	// 设置减速度
	virtual void cfgDec(float dec);
	// 设置电流参数
	// cra - 加速电流
	// crn - 工作电流
	// crh - 保持电流（尽量不要超过crn的一半）
	virtual void cfgCurrent(float cra, float crn, float crh);
	// 设置细分
	// mcs - 0 : 整步
	//       1 : 1/2
	//       2 : 1/4
	//       3 : 1/8
	//       4 : 1/16
	//       5 : 1/32
	//       6 : 1/64
	//       7 : 1/128
	//       8 : 1/256
	virtual void cfgMcs(int mcs);
	// 设置归零参数
	// zmd - 归零模式
	// snr - 归零传感器号（0-S1 1-S2 2-S3 3-S4 4-S5 5-S6）
	// osv - 开放状态的电平（无触发）
	// zsd - 归零速度（负值）
	// zsp - 归零后的安全位置（可不设置）
	virtual void cfgZero(int zmd, int snr, int osv, float zsd, int zsp);
	// 启动归零
	virtual void zeroStart();
	// 停止归零
	virtual void zeroStop();
	// 判断驱动器是否通讯在线
	virtual bool isOnline();
	// 设置为在线，重新参与通讯。如果通讯异常，自动变为离线
	virtual void online();
	// 设置为离线，不参与通讯
	virtual void offline();
	// 获取当前速度
	virtual float getCurSpd();
	// 获取当前位置
	virtual int32_t getCurPos();
	// 获取当前状态位
	virtual uint32_t getStatus();

};


class VSMD_DLL_API Vsmd
{
protected:

public:
	
	// 创建Vsmd对象
	// comPort - 串口号，例如："COM1"
	// baudrate - 波特率 （2400,921600）
	static Vsmd* createVsmd(char* comPort, int baudrate);
	// 创建驱动器对象
	// cid - 设备号
	//       0 : RS232驱动器对象
	//       1 - 32 : RS485驱动器对象
	virtual VsmdDevice* createDevice(int cid);
	// 打开vsmd对象
	virtual bool open();
	// 关闭vsmd对象
	virtual bool close();

};


