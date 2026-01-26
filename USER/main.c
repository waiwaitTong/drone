#include "main.h"
DECLARE_RM_OS_TASK(); // 申明创建的任务
DECLARE_RM_OS_FLAG(); // 定义事件标志组

int main(void)


{
	BSP_Init(); // 外设初始化
	CREATE_OS_TASK(StartTask); 
	vTaskStartScheduler(); // 开启任务调度
}

void StartTask(void *pvParameters)
	
{
	taskENTER_CRITICAL(); // 任务级进入临界区，关闭中断

	/*----------------------创建任务区-------------------*/

	CREATE_OS_TASK(SystemMonitorTask);
	CREATE_OS_TASK(Get_bmi_datatask);
	CREATE_OS_TASK(updatatask);
	CREATE_OS_TASK(gombol_control_task);
	CREATE_OS_TASK(SupplyPelletTask);
	CREATE_OS_TASK(FrictionWheelTask);
	CREATE_OS_TASK(Send_Data_Task);
	CREATE_OS_TASK(IMUCalibTask);
	CREATE_OS_TASK(Wireless_debugging);
	//	CREATE_OS_TASK(size_Task);
	CREATE_OS_TASK(RefereeSysTask);
	//	CREATE_OS_TASK(FWheelSampleTask);

	/*--------------------创建事件区--------------------*/
	IRQHandler = xEventGroupCreate();
	/*创建一个事件标志位，创建成功返回事件标志组的句柄*/
	/*IRQHandler为EventGroupHandle_t类型的变量保存返回值*/
	vTaskDelete(StartTask_Handler); // 删除开始任务
	taskEXIT_CRITICAL();			// 任务级退出临界区，打开中断
	// 任务调度是在中断中进行，所以关闭中断就能防止任务的调度
}
/*--------------------------------------
任务名称：SystemMonitorTask
任务功能：频率监视任务
--------------------------------------*/
void SystemMonitorTask(void *pvParameters) // 10
{
	while (1)
	{
		System_Monitor();
		vTaskDelay(1000);
	}
}

/*--------------------------------------
任务名称：gombol_control_task
任务功能：云台控制任务
--------------------------------------*/
void gombol_control_task(void *pvParameters) // 7
{
	while (1)
	{
		GimbalControl();
		vTaskDelay(1); // 1ms
	}
}

/*--------------------------------------
任务名称：SupplyBulletTask
任务功能：拨弹任务
--------------------------------------*/
void SupplyPelletTask(void *pvParameters) // 5
{
	while (1)
	{
		SupplyPelletControl();
		system_monitor.SupplyPelletTask_cnt++;
		vTaskDelay(1);
	}
}
/*--------------------------------------
任务名称：FrictionWheelTask()
任务功能：摩擦轮任务
--------------------------------------*/
void FrictionWheelTask(void *pvParameters) // 5
{
	while (1)
	{
		FrictionWheelControl();
		system_monitor.FrictionWheelTask_cnt++;
		vTaskDelay(1);
	}
}

/*--------------------------------------
任务名称：Send_Data_Task()
任务功能：向视觉更新数据
--------------------------------------*/
void Send_Data_Task(void *pvParameters) // 7
{
	while (1)
	{

		//PitchAngleUpdate(); // 向视觉更新数据//帧率为1000
		Vision_Tx_Protocol();
		system_monitor.USART4tx_cnt++;
		vTaskDelay(1);
	}
}
u8 NotifyValue;
/*--------------------------------------
任务名称：
任务功能：陀螺仪读取
--------------------------------------*/
void Get_bmi_datatask(void *pvParameters) // 6
{
	while (1)
	{
		//	    NotifyValue=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);//pdTRUE表示任务将等待直到收到通知，portMAX_DELAY表示等待时间的最大值
		if (BMI088_Cali_Flag)
		{
			Sensor_Data_Prepare();
			system_monitor.IMUSampleTask_cnt++;
		}
		vTaskDelay(1);
	}
}

void updatatask(void *pvParameters) // 7
{
	while (1)
	{
		if (BMI088_Cali_Flag)
		{
			IMU_Data_Deal();
			system_monitor.IMUUpdateTask_cnt++;
		}
		vTaskDelay(1);
	}
}
/*---------------------------------------------
任务名称：Wireless_debugging
任务功能：无线调试
----------------------------------------------*/
void Wireless_debugging(void *pvParameters) // 4
{
	while (1)
	{
		Send_Data_Update();
		system_monitor.Wireless_debugging_cnt++;
		vTaskDelay(1);
	}
}
/*--------------------------------------
 任务名称：RefereeSysTask()
 任务功能：向裁判系统发送数据
--------------------------------------*/
SCHAR8 Referee_control = 1;
void RefereeSysTask(void *pvParameters) // 4
{
	while (1)
	{
		switch (Referee_control)
		{
		case 1:
			Send_Missile_Data();
			Referee_control++;
			break;
		case 2:
			Send_Custom_Data();
			Referee_control = 1;
			break;
		default:
			break;
		}
		system_monitor.USART2tx_cnt++;
		vTaskDelay(30);
	}
}

/*--------------------------------------
任务功能：IMU标定
--------------------------------------*/
void IMUCalibTask(void *pvParameters) // 5
{
	while (1)
	{
		if (!BMI088_Cali_Flag) // 云控没有标定时执行
		{
			BMI088_Cali_Flag = GetOffset();
			system_monitor.IMUCalibrationTask_cnt++;
		}
        // if(Manual_flag)//手动标定
        // {
        //     Manual_Fix();
        // }
		vTaskDelay(1);
	}
}

void size_Task(void *pvParameters)
{
	while (1)
	{
		Z_Offset_Debugger();
		vTaskDelay(1);
	}
}
