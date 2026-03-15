/**********************************************************************************************************
版权声明： RoboMasters(HITCRT_iHiter 战队)
文件名： rm_algorithm.c
最近修改日期： 2018/03/26
版本： 3.1
-----------------------------------------------------------------------------------------------------------
模块描述：该模块定义了RM常用的算法。
-----------------------------------------------------------------------------------------------------------
修改记录：
作者                              时间         版本   说明
彭季超、彭毓兴、张冰、李四林    2016/01/25     1.0     建立此文件
FengYun                         2017/03/08	   2.0	   更新了PID算法以适应新的PID结构体，将原先增量式算法改
                                                       为位置式，完善了改进型PID计算式
SOLDIER                         2018/01/27     3.0     更改遇限削弱PID算法，针对云台辅助瞄准输入信号复杂，阶跃与
                                                       斜坡信号并存，针对底盘功率限制可能存在较长
													   时间静差导致速度环崩溃，缩小积分范围。
													   更改抗积分饱和算法，抑制拨弹电机几个周期出现超调的情况。
SOLDIER                         2018/03/26     3.1     加入滤波器，减少MPU6050高频噪声（第三代车取消MPU6050）
***********************************************************************************************************/

#include "rm_algorithm.h"
#include "math.h"
#include "string.h"
/*位置式PID算法 u(k)=Kp*E(K)+Ki*sum(E(K))+Kd*(E(K)-E(K-1))*/

/*******************************************************************
函数名称：CalPID(ST_PID *pstPid)
函数功能：普通的PID算法计算PID量
备    注：
********************************************************************/
void CalPID(ST_PID *pstPid)
{
    pstPid->fpE = pstPid->fpDes - pstPid->fpFB;//计算当前偏差
	if(fabs(pstPid->fpE) <= pstPid->fpEMin)//偏差死区限制
	{
	    pstPid->fpE = 0;
	}
	pstPid->fpSumE += pstPid->fpE;
	/*位置式PID计算公式*/
	pstPid->fpU = pstPid->fpKp * pstPid->fpE 
	            + pstPid->fpKi * pstPid->fpSumE 
				+ pstPid->fpKd * (pstPid->fpE - pstPid->fpPreE);
	pstPid->fpPreE = pstPid->fpE;//保存本次偏差
    /*PID运算输出限幅*/
    if(pstPid->fpU > pstPid->fpUMax)
	{
	    pstPid->fpU = pstPid->fpUMax;
	}
	else if(pstPid->fpU < -pstPid->fpUMax)
	{
	    pstPid->fpU = -pstPid->fpUMax;
	}

}

/*******************************************************************
函数名称：CalIWeakenPID(ST_PID *pstPid)
函数功能：遇限削弱积分PID改进算法计算PID量
备    注：
********************************************************************/
void CalIWeakenPID(ST_PID *pstPid)
{

	pstPid->fpE=pstPid->fpDes-pstPid->fpFB;//计算当前偏差
	

	if(((pstPid->fpU <= pstPid->fpUMax && pstPid->fpE > 0) || (pstPid->fpU >= -pstPid->fpUMax && pstPid->fpE < 0)) \
		    && abs_fl(pstPid->fpE) < pstPid->fpEMin)
	{
		pstPid->fpSumE += pstPid->fpE;//计算偏差累积
	}
	if(pstPid->fpKipre!=pstPid->fpKi)
	{
		pstPid->fpSumE=0;
	}
	pstPid->fpKipre=pstPid->fpKi;
	pstPid->fpSumE = Clip(pstPid->fpSumE, -pstPid->fpEiMax, pstPid->fpEiMax);
	pstPid->fpUi = pstPid->fpKi * pstPid->fpSumE;
	
//	pstPid->fpUp = Clip(pstPid->fpKp * log(pstPid->fpE-1), -pstPid->fpEpMax, pstPid->fpEpMax);

	pstPid->fpUp = Clip(pstPid->fpKp * pstPid->fpE, -pstPid->fpEpMax, pstPid->fpEpMax);
	pstPid->fpUd = Clip(pstPid->fpKd * (pstPid->fpE - pstPid->fpPreE), -pstPid->fpEdMax, pstPid->fpEdMax);
	
	/*位置式PID计算公式*/
	pstPid->fpU = pstPid->fpUp + pstPid->fpUi + pstPid->fpUd;
	
	pstPid->fpPreE = pstPid->fpE;//保存本次偏差
	
    /*PID运算输出限幅*/	
	pstPid->fpU = Clip(pstPid->fpU, -pstPid->fpUMax, pstPid->fpUMax);
}

/*******************************************************************
函数名称：TD_Function(TD *ptd)
函数功能：跟踪微分器
备    注：
********************************************************************/
void TD_Function(TD *ptd)
{
	/***/
	float d,d0,y,a0,a=0;
	ptd->x = ptd->x1 - ptd->aim;
	d = ptd->r*ptd->h;
    d0=ptd->h * d;
    y = ptd->x + ptd->h*ptd->x2;
	a0 = sqrt(d*d+8*ptd->r*abs_fl(y));
	
    if(abs_fl(y)>d0)	
		a = ptd->x2+(a0-d)*Sign_Judge(y)/2;
	else
		a = ptd->x2 + y/ptd->h;
	
    if(abs_fl(a)>d)
		y=-1*ptd->r*Sign_Judge(a);
	else
		y=-1*ptd->r*a/d;

	ptd->x1 +=  0.001f*ptd->x2;
	ptd->x2 +=  0.001f*y; 
}

float abs_fl(float value) //绝对值
{
	if(value>0) return value;
	else if(value<0) return -value;
	else return 0;
}


void Kalman_Filter(kalman_filter *pkf)
{
    pkf->x_mid = pkf->x_last;
	pkf->p_mid = pkf->p_last + pkf->Q;
	pkf->K = pkf->p_mid/(pkf->p_mid + pkf->R);
	pkf->x_now = pkf->x_mid + pkf->K*(pkf->raw_value - pkf->x_mid);
	pkf->p_now = (1-pkf->K)*pkf->p_mid;
	
	pkf->x_last = pkf->x_now;
	pkf->p_last = pkf->p_now;
}


/*******************************************************************
函数名称：CalSMC(ST_SMC *pStSMC)
函数功能：滑模控制算法
备    注：
********************************************************************/
void CalSMC(ST_SMC *pStSMC)
{
	pStSMC->TD.aim = pStSMC->fpDes;	//通过TD微分控制器算出滤波后的位置与速度目标值
	TD_Function(&pStSMC->TD);
	pStSMC->fpE = pStSMC->TD.x1 - pStSMC->fpFB;//为文档中的x1=wref-wm
//    if(fabs(pStSMC->fpFB)>fabs(pStSMC->TD.x1))
//        pStSMC->TD.x1 = pStSMC->fpFB;

	pStSMC->fpU = 1 / pStSMC->b * (pStSMC->TD.x2
								   + pStSMC->eps * SMC_SatFunc(pStSMC->fpE, pStSMC->dead)
								   + pStSMC->gain * pStSMC->fpE);//选择了c滑模面的控制参数为1，c越大，系统收敛的越快，但是超调量也会增加
	pStSMC->fpU = Clip(pStSMC->fpU, -pStSMC->fpUMax, pStSMC->fpUMax);
}


float SMC_SatFunc(float in, float d)
{
	if(fabs(in) >= d)
		return Sign_Judge(in);
	else
		return in / d;
}

