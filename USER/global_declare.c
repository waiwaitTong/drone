#include "global_declare.h"

SYSTEM_MONITOR system_monitor = {0};			  // 系统监视器
ST_TESTFLAG g_stTestFlag = {FALSE, FALSE, FALSE}; // 任务测试标志
u8 BMI088_Cali_Flag = 1;						  // 云控标定标志位
u8 Manual_flag=0;//手动标定

// bmi陀螺仪
FP32 BMIPitchSpeed;
FP32 BMIYawSpeed;
IMU_MODE imu_mode = INIT;
/*限幅变量*/
FP32 Pitch_Limit_Up_imu = 10;   // 10
FP32 Pitch_Limit_Down_imu = -25; //-25
FP32 Yaw_Limit_Up_imu = 30;
FP32 Yaw_Limit_Down_imu = -25;
FP32 Roll_Limit_Up_imu = 15;
FP32 Roll_Limit_Down_imu = -15;
/*码盘值*/
FP32 Pitch_encode = 0;
FP32 Yaw_encode = 0;
FP32 Roll_encode = 0;
/*转矩电流值*/
FP32 Pitch_current = 0;
FP32 Yaw_current = 0;
FP32 Roll_current = 0;
/*云台两组pid*/
										
// 阶跃
//ST_PID Pitch_SpeedPID_3 = {0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500};
//ST_PID Pitch_PosPID_3 = {0, 0, 15, 0.01, 200, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 500, 500};
//TD pitch_td_3 = {0, 0, 0, 3000, 0.001, 0};

//ST_PID Yaw_SpeedPID_3 = {0, 0, 150, 0, 500, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500}; 
//ST_PID Yaw_PosPID_3 = {0, 0, 30, 0.035, 800, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 500, 500};	 
//TD yaw_td_3 = {0, 0, 0, 3000, 0.001, 0};	

/*5.21*/
//ST_PID Pitch_SpeedPID = {0, 0, 120, 0, 0, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500};
//ST_PID Pitch_PosPID = {0, 0, 30, 0.07, 200, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 500, 500};
//TD pitch_td = {0, 0, 0, 1600, 0.001, 0};

//ST_PID Yaw_SpeedPID = {0, 0, 110, 0, 1500, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500}; 
//ST_PID Yaw_PosPID = {0, 0, 30, 0.03, 2500, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 500, 500};   
//TD yaw_td = {0, 0, 0, 1100, 0.001, 0};		

//ST_PID Roll_SpeedPID = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25000, 20000, 5000, 5000, 500}; 
//ST_PID Roll_PosPID = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 20000, 3000, 500, 500, 500};   

//pitch不朝下
ST_PID Pitch_SpeedPID_1 = {0, 0, 55, 0, 1000, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500}; 
ST_PID Pitch_PosPID_1 = {0, 0, 28, 0.005, 1000, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 3000, 1};   
TD pitch_td_1 = {0, 0, 0, 10000, 0.001, 0};



ST_PID Yaw_SpeedPID_1 ={0, 0, 55, 0, 1000, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500}; 
ST_PID Yaw_PosPID_1 = {0, 0, 30, 0.06, 1000, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 3000, 1};     
TD yaw_td_1 = {0, 0, 0, 10000, 0.001, 0};	

// pitch朝下时
ST_PID Pitch_SpeedPID_2 = {0, 0, 6.5, 0, 120, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500}; 
ST_PID Pitch_PosPID_2 = {0, 0, 30, 0.001, 50, 0, 0, 0, 0, 0, 0, 0, 2048, 2000, 500, 500, 2};
TD pitch_td_2 = {0, 0, 0, 10000, 0.001, 0};

ST_PID Yaw_SpeedPID_2 = {0, 0, 50, 0, 800, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500};	
ST_PID Yaw_PosPID_2 = {0, 0, 26, 0.055, 800, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 3000, 1}; 
TD yaw_td_2 = {0, 0, 0, 1500, 0.001, 0};////////zuixia






// 老
//ST_PID Pitch_SpeedPID_3 = {0, 0, 5, 0, 40, 0, 0, 0, 0, 0, 0, 0, 2048, 2000, 2000, 2000, 500};
//ST_PID Pitch_PosPID_3 = {0, 0, 20, 0.001, 75, 0, 0, 0, 0, 0, 0, 0, 2048, 2000, 500, 500, 2};
//TD pitch_td_3 = {0, 0, 0, 10000, 0.001, 0};
ST_PID Pitch_SpeedPID_3 = {0, 0, 7, 0, 75, 0, 0, 0, 0, 0, 0, 0, 2048, 2000, 2000, 2000, 500};
ST_PID Pitch_PosPID_3 = {0, 0, 30, 0.05, 120, 0, 0, 0, 0, 0, 0, 0, 2048, 2000, 500, 500, 2};
TD pitch_td_3 = {0, 0, 0, 10000, 0.001, 0};

ST_PID Yaw_SpeedPID_3 = {0, 0, 110, 0, 800, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500};	
ST_PID Yaw_PosPID_3 = {0, 0, 38, 0.06, 900, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 3000, 1}; 
TD yaw_td_3 = {0, 0, 0, 1500, 0.001, 0};




//7.9
//ST_PID Pitch_SpeedPID = {0, 0, 120, 0, 0, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500};
//ST_PID Pitch_PosPID = {0, 0, 25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 500, 500};
//TD pitch_td = {0, 0, 0, 1600, 0.001, 0};

//ST_PID Yaw_SpeedPID = {0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500}; 
//ST_PID Yaw_PosPID = {0, 0, 15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 500, 500};   
//TD yaw_td = {0, 0, 0, 1100, 0.001, 0};		

//ST_PID Roll_SpeedPID = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25000, 20000, 5000, 5000, 500}; 
//ST_PID Roll_PosPID = {0, 0, -70, -0.3, 0, 0, 0, 0, 0, 0, 0, 0, 4000, 3000, 500, 500, 500};   

//7.10
//ST_PID Pitch_SpeedPID = {0, 0, 170, 0, 600, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500};
//ST_PID Pitch_PosPID = {0, 0, 40, 0.01, 300, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 500, 500};
//TD pitch_td = {0, 0, 0, 1600, 0.001, 0};

//ST_PID Yaw_SpeedPID = {0, 0, 130, 0, 800, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500}; 
//ST_PID Yaw_PosPID = {0, 0, 40, 0.001, 1000, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 500, 500};   
//TD yaw_td = {0, 0, 0, 1100, 0.001, 0};		

//ST_PID Roll_SpeedPID = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25000, 20000, 5000, 5000, 500}; 
//ST_PID Roll_PosPID = {0, 0, -70, -0.3, 0, 0, 0, 0, 0, 0, 0, 0, 4000, 3000, 500, 500, 500};   

//7.16
//ST_PID Pitch_SpeedPID = {0, 0, 150, 0, 0, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500};
//ST_PID Pitch_PosPID = {0, 0, 40, 0.025, 200, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 500, 500};
//TD pitch_td = {0, 0, 0, 1600, 0.001, 0};

//ST_PID Yaw_SpeedPID = {0, 0, 150, 0, 800, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500}; 
//ST_PID Yaw_PosPID = {0, 0, 40, 0.001, 1000, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 500, 500};   
//TD yaw_td = {0, 0, 0, 1100, 0.001, 0};		

////ST_PID Roll_SpeedPID = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 20000, 5000, 5000, 500}; 
////ST_PID Roll_PosPID = {0, 0, -70, -0.3, 0, 0, 0, 0, 0, 0, 0, 0, 4000, 3000, 500, 500, 500};   

//ST_PID Roll_SpeedPID = {0, 0, 0.3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 8, 3, 3,3}; 
//ST_PID Roll_PosPID = {0, 0, -10, 0.001, 0, 0, 0, 0, 0, 0, 0, 0, 30, 25, 2, 2, 2};   

//7.18
//ST_PID Pitch_SpeedPID = {0, 0, 140, 0, 0, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500};
//ST_PID Pitch_PosPID = {0, 0, 35, 0.025, 200, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 500, 500};
//TD pitch_td = {0, 0, 0, 1600, 0.001, 0};

//ST_PID Yaw_SpeedPID = {0, 0, 130, 0, 800, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500}; 
//ST_PID Yaw_PosPID = {0, 0, 40, 0.001, 1000, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 500, 500};   
//TD yaw_td = {0, 0, 0, 1100, 0.001, 0};		

////ST_PID Roll_SpeedPID = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 20000, 5000, 5000, 500}; 
////ST_PID Roll_PosPID = {0, 0, -70, -0.3, 0, 0, 0, 0, 0, 0, 0, 0, 4000, 3000, 500, 500, 500};   

//ST_PID Roll_SpeedPID = {0, 0, 0.3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 8, 3, 3,3}; 
//ST_PID Roll_PosPID = {0, 0, -10, 0.005, 0, 0, 0, 0, 0, 0, 0, 0, 30, 25, 2, 2, 2};   

////7.20
//ST_PID Pitch_SpeedPID = {0, 0, 90, 0, 0, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500};
//ST_PID Pitch_PosPID = {0, 0, 35, 0.025, 300, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 500, 500};
//TD pitch_td = {0, 0, 0, 1600, 0.001, 0};

//ST_PID Yaw_SpeedPID = {0, 0, 130, 0, 800, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500}; 
//ST_PID Yaw_PosPID = {0, 0, 40, 0.001, 1000, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 500, 500};   
//TD yaw_td = {0, 0, 0, 1100, 0.001, 0};		

////ST_PID Roll_SpeedPID = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 20000, 5000, 5000, 500}; 
////ST_PID Roll_PosPID = {0, 0, -70, -0.3, 0, 0, 0, 0, 0, 0, 0, 0, 4000, 3000, 500, 500, 500};   

//ST_PID Roll_SpeedPID = {0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 8, 3, 3,3}; 
//ST_PID Roll_PosPID = {0, 0, -40, 0.005, 0, 0, 0, 0, 0, 0, 0, 0, 30, 25, 2, 2, 2}; 

//7.21
//ST_PID Pitch_SpeedPID = {0, 0, 100, 0, 200, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500};
//ST_PID Pitch_PosPID = {0, 0, 40, 0.025, 300, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 500, 500};
//TD pitch_td = {0, 0, 0, 1600, 0.001, 0};




/////////////////////1.12
//ST_PID Pitch_SpeedPID = {0, 0, 100, 0, 400, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500};
//ST_PID Pitch_PosPID = {0, 0, 30, 0.2, 400, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 5000, 5000, 1};
//TD pitch_td = {0, 0, 0, 1800, 0.001, 0};



//ST_PID Yaw_SpeedPID = {0, 0, 160, 0, 1600, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500}; 
//ST_PID Yaw_PosPID = {0, 0, 55, 0.03, 1200, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 3000, 1};   
//TD yaw_td = {0, 0, 0, 1300, 0.001, 0};



/////////////////////5.15

//ST_PID Pitch_SpeedPID ={0, 0, 11, 0, 100, 0, 0, 0, 0, 0, 0, 0, 2048, 2000, 2000, 2000, 500};
//ST_PID Pitch_PosPID = {0, 0, 28, 0.001, 110, 0, 0, 0, 0, 0, 0, 0, 2048, 2000, 500, 500, 2};
//TD pitch_td = {0, 0, 0, 10000, 0.001, 0};
ST_PID Pitch_SpeedPID ={0, 0, 6, 0, 40, 0, 0, 0, 0, 0, 0, 0, 2048, 2000, 2000, 2000, 500};
ST_PID Pitch_PosPID = {0, 0, 20, 0.06, 75, 0, 0, 0, 0, 0, 0, 0, 2048, 2000, 1000, 500, 4};
TD pitch_td = {0, 0, 0, 10000, 0.001, 0};


ST_PID Yaw_SpeedPID ={0, 0, 45, 0, 150, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500}; 
ST_PID Yaw_PosPID = {0, 0, 30, 0.005, 100, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 3000, 1};     
TD yaw_td = {0, 0, 0, 10000, 0.001, 0};




/////////////////1.12
//ST_PID Pitch_SpeedPID = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500};
//ST_PID Pitch_PosPID = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 5000, 5000, 1};
//TD pitch_td = {0, 0, 0, 1800, 0.001, 0};



//ST_PID Yaw_SpeedPID = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500}; 
//ST_PID Yaw_PosPID = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 3000, 1};   
//TD yaw_td = {0, 0, 0, 1400, 0.001, 0};

//////cAN

//ST_PID Pitch_SpeedPID = {0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500};
//ST_PID Pitch_PosPID = {0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 500, 1};
//TD pitch_td = {0, 0, 0, 1800, 0.001, 0};



//ST_PID Yaw_SpeedPID = {0, 0, 150, 0, 0, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500}; 
//ST_PID Yaw_PosPID = {0, 0, 30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 500, 1};   
//TD yaw_td = {0, 0, 0, 1400, 0.001, 0};






//ST_PID Pitch_SpeedPID = {0, 0, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 2000, 2000, 2000, 500};
//ST_PID Pitch_PosPID = {0, 0, 6, 0.0001, 3, 0, 0, 0, 0, 0, 0, 0, 2048, 2000, 500, 500, 500};
//TD pitch_td = {0, 0, 0, 3000, 0.001, 0};

///*原参
//ST_PID Yaw_SpeedPID = {0, 0, 140, 0, 800, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500}; 
//ST_PID Yaw_PosPID = {0, 0, 45, 0.005, 1600, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 500, 500};   
//TD yaw_td = {0, 0, 0, 1300, 0.001, 0};		*/

//ST_PID Yaw_SpeedPID = {0, 0, 130, 0, 0, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500}; 
//ST_PID Yaw_PosPID = {0, 0, 11, 0.01, 0, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 500, 500};   
//TD yaw_td = {0, 0, 0, 2500, 0.001, 0};



//ST_PID Roll_SpeedPID = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 20000, 5000, 5000, 500}; 
//ST_PID Roll_PosPID = {0, 0, -70, -0.3, 0, 0, 0, 0, 0, 0, 0, 0, 4000, 3000, 500, 500, 500};   

//ST_PID Roll_SpeedPID = {0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 8, 3, 3,3}; 
//ST_PID Roll_PosPID = {0, 0, -40, 0.005, 0, 0, 0, 0, 0, 0, 0, 0, 30, 25, 2, 2, 2};   

ST_PID Roll_SpeedPID = {0, 0, 0.0045, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 8, 3, 3,3}; 
ST_PID Roll_PosPID = {0, 0, -110, -0.1, 0, 0, 0, 0, 0, 0, 0, 0, 30, 25, 10, 2, 2};   
    
// 响应90，抖动0.7
//ST_PID Yaw_SpeedPID = {0, 0, 80, 0, 1200, 0, 0, 0, 0, 0, 0, 0, 30000, 28000, 5000, 5000, 500}; 
//ST_PID Yaw_PosPID = {0, 0, 15, 0.03, 0, 0, 0, 0, 0, 0, 0, 0, 29000, 3000, 500, 500, 500};   
//TD yaw_td = {0, 0, 0, 3000, 0.001, 0};		
/*
	FP32 fpDes; // 控制变量目标值
	FP32 fpFB;	// 控制变量反馈值

	FP32 fpKp; // 比例系数Kp
	FP32 fpKi; // 积分系数Ki
	FP32 fpKd; // 微分系数Kd

	FP32 fpUp; // 比例输出
	FP32 fpUi; // 积分输出
	FP32 fpUd; // 微分输出

	FP32 fpE;	 // 本次偏差
	FP32 fpPreE; // 上次偏差
	FP32 fpSumE; // 总偏差
	FP32 fpU;	 // 本次PID运算结果

	FP32 fpUMax;  // PID运算后输出最大值及做遇限削弱时的上限值13
	FP32 fpEpMax; // 比例项输出最大值
	FP32 fpEiMax; // 积分项输出最大值
	FP32 fpEdMax; // 微分项输出最大值16
	FP32 fpEMin;  // 积分上限
	FP32 fpKipre;
	*/

FP32 GravityCompensation = 0; // 重力补偿
s16 YawCurrent = 0;			  // yaw电流
s16 PitchCurrent = 0;		  // pitch电流
FP32 RollCurrent = 0;	
FP32 PitchPos_Reference = 0;  // 目标值
FP32 YawPos_Reference = 0;
FP32 RollPos_Reference = 0;

FP32 Visionrx_freq_data_p = 0;

ST_LPF pitch_speed_lpf = {0, 0, 0, 100, 0.001}; // 100
ST_LPF pitch_pos_lpf = {0, 0, 0, 100, 0.001};	// 100

kalman_filter pitch_kf = {0, 0, 0, 1, 0, 0, 0, 0, 100, 1}; // 位置环目标值滤波
kalman_filter yaw_kf = {0, 0, 0, 1, 0, 0, 0, 0, 100, 1};
kalman_filter pitchfb_kf = {0, 0, 0, 1, 0, 0, 0, 0, 10, 1};
kalman_filter yawfb_kf = {0, 0, 0, 1, 0, 0, 0, 0, 200, 1};
kalman_filter rollfb_kf = {0, 0, 0, 1, 0, 0, 0, 0, 10, 1};

kalman_filter pitch_speed_kf = {0, 0, 0, 1, 0, 0, 0, 0, 20, 1};
kalman_filter yaw_speed_kf = {0, 0, 0, 1, 0, 0, 0, 0, 20, 1};

// 遥控器
ST_DBUS DR16_rec;					  // 接收机接收到的数据结构体
EM_OPERATION_MODE g_emOperation_Mode; // 机器人控制方式

// 摩擦轮
bool FrictionWheel_ReadyFLAG = FALSE; // 摩擦轮到达目标值FLAG
bool FrictionWheel_Ready = FALSE;	  // 摩擦轮开关
u16 FrictionWheel_Ready_cnt = 0;

volatile ST_ENCODER g_Encoder_1SMC = {0, 0, 0, 0, 1, 8192, 0};
volatile ST_ENCODER g_Encoder_2SMC = {0, 0, 0, 0, 1, 8192, 0};

ST_SMC g_stFriction1SMC = {
	.b = 4.9,
	.eps = 7500,
	.dead = 5,
	.gain = 50,
	.fpDes = 0,
	.fpFB = 0,
	.fpU = 0,
	.fpUMax = 16384, /*摩擦轮反馈结构体*/

	.TD.h = 0.001,
	.TD.r = 7000,////////
	.TD.aim = 0,
	.TD.x = 0,
	.TD.x1 = 0,
	.TD.x2 = 0};
ST_SMC g_stFriction2SMC = {
	.b = 4.9,
	.eps = 7500,
	.dead = 5,
	.gain = 50,
	.fpDes = 0,
	.fpFB = 0,
	.fpU = 0,
	.fpUMax = 16384,

	.TD.h = 0.001,
	.TD.r = 7000,  //////////////////
	.TD.aim = 0,
	.TD.x = 0,
	.TD.x1 = 0,
	.TD.x2 = 0};

// ST_SMC g_stFriction1SMC = ST_SMC_INIT(15000,4.9,7000,50,5,7000,0.001,0.001);  //滑模控制
// ST_SMC g_stFriction2SMC = ST_SMC_INIT(15000,4.9,7500,54,5,7000,0.001,0.001);  //滑模控制
//
// #define  ST_SMC_INIT(fpUMax,b,eps,gain,dead,TD_r,TD_h,TD_T) \
//        {0,0,0,0,fpUMax,b,eps,gain,dead,0,0,0,TD_r,TD_h,TD_T,0}
/*typedef struct
{
	FP32 fpDes;	 // 控制变量目标值
	FP32 fpFB;	 // 控制变量反馈值
	FP32 fpE;	 // 本次偏差
	FP32 fpU;	 // 本次运算结果
	FP32 fpUMax; // 输出上限

	// SMC参数
	FP32 b;
	FP32 eps;
	FP32 gain;
	FP32 dead;
	TD TD;
} ST_SMC;*/
/*-------------------------------云台拨弹电机------------------------------------------------*/

volatile ST_ENCODER g_stShooterEncoder = {0, 0, 0, 0, 19.20321, 8192, 0};							 // 拨弹电机编码盘结构体
//volatile ST_ENCODER g_stShooterEncoder = {0, 0, 0, 0, 51, 8192, 0};							 // 拨弹电机编码盘结构体
//ST_PID Supply_PosPID = {0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10000, 8000, 100, 0, 100};	 // 拨弹电机位置环结构体
//ST_PID Supply_SpeedPID = {0, 0, 210, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10000, 10000, 0, 10000, 20}; // 拨弹电机速度环结构体
ST_PID Supply_PosPID =   {0, 0, 0.13, 0.002, 0, 0, 0, 0, 0, 0, 0, 0, 10000, 8000, 5000, 0, 100};	 // 拨弹电机位置环结构体 3508
//ST_PID Supply_SpeedPID = {0, 0, 200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 16384, 16000, 0, 10000, 20}; // 拨弹电机速度环结构体
ST_PID Supply_SpeedPID = {0, 0, 350, 0, 0, 0, 0, 0, 0, 0, 0, 0, 16384, 16384, 2000, 10000, 20}; // 拨弹电机速度环结构体

/*typedef struct
{
	FP32 fpDes; // 控制变量目标值
	FP32 fpFB;	// 控制变量反馈值

	FP32 fpKp; // 比例系数Kp
	FP32 fpKi; // 积分系数Ki
	FP32 fpKd; // 微分系数Kd

	FP32 fpUp; // 比例输出
	FP32 fpUi; // 积分输出
	FP32 fpUd; // 微分输出

	FP32 fpE;	 // 本次偏差
	FP32 fpPreE; // 上次偏差
	FP32 fpSumE; // 总偏差
	FP32 fpU;	 // 本次PID运算结果

	FP32 fpUMax;  // PID运算后输出最大值及做遇限削弱时的上限值
	FP32 fpEpMax; // 比例项输出最大值
	FP32 fpEiMax; // 积分项输出最大值
	FP32 fpEdMax; // 微分项输出最大值16
	FP32 fpEMin;  // 积分上限
	FP32 fpKipre;// 上次i值
} ST_PID;*/
Supply_Status Supply_Motor_Status = May_stop;												 // 卡弹未确定
Gimbal_Control_Mode Gimbal_Control = NO_Aim_Mode;											 // 是否开辅瞄标志

/*-------------------------------------视觉通信协议--------------------------------------------
					  0x55 0x00 ID NUM NUM*float 0x00 0xAA
----------------------------------------------------------------------------------------------*/
ST_VISION G_ST_Vision = {0};
UN_AIM_DATA unAimData;
float yaw_fileter = 0;
float pitch_fileter = 0;
float roll_fileter = 0;


float Pitch_E=0;
float Yaw_E=0;


/*-------------------------------------键鼠--------------------------------------------*/

bool PRESSED_W = FALSE;
bool PRESSED_S = FALSE;
bool PRESSED_A = FALSE;
bool PRESSED_D = FALSE;
bool PRESSED_SHIFT = FALSE;
bool PRESSED_CTRL = FALSE;
bool PRESSED_Q = FALSE;
bool PRESSED_E = FALSE;
bool PRESSED_R = FALSE;
bool PRESSED_F = FALSE;
bool PRESSED_G = FALSE;
bool PRESSED_Z = FALSE;
bool PRESSED_X = FALSE;
bool PRESSED_C = FALSE;
bool PRESSED_V = FALSE;
bool PRESSED_B = FALSE;
bool PRESSED_ML = FALSE;
bool PRESSED_MR = FALSE;

DBUS_BUFF Mouse_X = {0};
DBUS_BUFF Mouse_Y = {0};

/*---------------------------------裁判系统---------------------------------------------------*/
ST_RSYS_MONITOR RSYS_Monitor = {0};
ext_shoot_data_t Shoot_Data = {0};											// 射击状态                 1
ext_game_robot_pos_t Drone_Pos = {0};										// 空间位置                 1
ext_power_heat_data_t Shoot_Power = {0};									// 当前射击热量值            1
ext_game_robot_state_t Drone_State = {0};									// 发射机构供电状态 友方阵营 1
UN_Custom_Robo Custom_Robo_Data = {0};										// 机器人自定义交互数据
UN_Custom_Client Custom_Client_Data = {0};									// 客户端显示数据
ext_bullet_remaining_t Ammo_remain = {0};									// 剩余子弹数   ID:0208    1
ext_game_robot_HP_t Robot_HP = {0};											// 机器人血量数据  0x0003  1
ext_game_status_t Game_Status = {0};										// 比赛状态    0x0001;    1
ext_student_interactive_header_data_t G_ST_Student_Interactive_Header_Data; // 交互数据接收信息
robot_interactive_data_t G_ST_Robot_Interactive_Data;						// 1.交互数据
ext_client_custom_graphic_delete_t G_ST_Client_Custom_Graphic_Delete;		// 2.客户端删除图形
ext_client_custom_graphic_single_t G_ST_Client_Custom_Graphic_Single;		// 3.客户端绘制一个图形
ext_client_custom_graphic_double_t G_ST_Client_Custom_Graphic_Double;		// 4.客户端绘制二个图形
ext_client_custom_graphic_five_t G_ST_Client_Custom_Graphic_Five;			// 5.客户端绘制五个图形
ext_client_custom_graphic_seven_t G_ST_Client_Custom_Graphic_Seven;			// 6.客户端绘制七个图形
ext_client_custom_graphic_single_t G_ST_Client_Custom_Graphic_Single_Rectangle;
ext_client_custom_character_t G_ST_Client_Custom_Character;
u8 SendData_Sentinel[3];

ext_client_custom_character_t G_ST_Client_Custom_Character; // 7.客户端绘制字符
ext_missile_data_t DataFromMissile = {0};					// 来自导弹的数据
ext_sentinel_data_t DataFromSentinel = {0};					// 来自哨兵的数据
ext_ground_data_t DataFromSoldier3 = {0};					// 来自步兵3的数据
ext_ground_data_t DataFromSoldier4 = {0};					// 来自步兵4的数据
ext_ground_data_t DataFromSoldier5 = {0};					// 来自步兵5的数据
ext_ground_data_t DataFromHero = {0};						// 来自英雄的数据

UCHAR8 Sending_num = 2;					  // 飞镖发送数目
UCHAR8 picture_num = 1;					  // 雷达界面
EM_AIM_MODE g_emAim_Mode = Aim_Zero_Mode; // 辅瞄模式切换
UCHAR8 sentinel_change_id = 0;			  // 哨兵是否改变击打红蓝方

//remote_control_t remote_control_image_transmission; // 图传发送遥控器指令
remote_control_keyboard_mouset_t remote_control_image_transmission;
/*---------------------------------无线调试---------------------------------------------------*/
Send_Data_Vofa Send_vofa = {0};

/*-----------------------------------步兵号码选择----------------------------------------------*/
int infantry3_flag = 0;
int infantry4_flag = 0;
int infantry5_flag = 0;
int infantry_number = 0;

u8 vision_lock = 0;


/*LK电机*/
Motor_LK_7010 ARM_LK_Motor1= {.LK_motor_encoder = {.Number = 65535.f}};
Motor_LK_7010 ARM_LK_Motor2= {.LK_motor_encoder = {.Number = 65535.f}};
