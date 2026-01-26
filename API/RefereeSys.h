#ifndef __REFEREESYS_H
#define __REFEREESYS_H
#include "global_declare.h"
#include "stdio.h"
#include "string.h"
#include "GlobalUse_Basic_Function.h"
#include "gimbol_control_task.h"

#define USART2_RX_FREE             0
#define USART2_RX_Length           1
#define USART2_RX_CRC8             2
#define USART2_RX_CmdID            3
#define USART2_RX_Data             4
#define USART2_RX_CRC16            5

#define USART6_RX_FREE             0
#define USART6_RX_Length           1
#define USART6_RX_CRC8             2
#define USART6_RX_CmdID            3
#define USART6_RX_Data             4
#define USART6_RX_CRC16            5

#define Game_Status_ID              0x0001//TODO DONE//比赛状态数据
#define GameResultID                0x0002
#define Robot_HP_ID                 0x0003
#define DartStatusID                0x0004
#define ICRABuffDebuffZoneStatusID  0x0005
#define EventDataID                 0x0101//TODO 没用
#define SupplyProjectileActionID    0x0102//TODO 没用
#define RefereeWaringID             0x0104//TODO 没用
#define DartRemainingTimeID         0x0105//TODO 没用
#define Robot_State_ID              0x0201//TODO DONE //机器人性能体系 todo
#define Power_Data_ID   			0x0202//实时射击热量数据 todo
#define Robot_Pos_ID    			0x0203//TODO  DONE
#define BuffID                      0x0204
#define Drone_Energy_ID 			0x0205//没用
#define RobotHurtID                 0x0206
#define Shoot_Data_ID  		    	0x0207//实时射击数据 todo
#define Ammo_Remain_ID  			0x0208//允许发弹量	 todo
#define RFIDStatusID                0x0209//TODO  没用
#define DartClientCmdID             0x020A//TODO  没用
#define Robo_InteractiveCmdID       0x0301//TODO //机器人交互数据 todo
#define ROBOT_COMMAND_CMD_ID        0x0304//键鼠遥控数据 todo


#define Sender_ID   Drone_State.robot_id
#define Receiver_Sentinel_ID   Drone_State.robot_id+1
#define Receiver_Missile_ID    Drone_State.robot_id+2
#define Receiver_Radar_ID      Drone_State.robot_id+3
#define Receiver_Client_ID (Drone_State.robot_id == 6)?0x0106:0x016A


#define MASK1_ON 	Custom_Client_Data.stCustomClientInfo.masks|= (0x01<<(1-1))
#define MASK2_ON 	Custom_Client_Data.stCustomClientInfo.masks|= (0x01<<(2-1))
#define MASK3_ON 	Custom_Client_Data.stCustomClientInfo.masks|= (0x01<<(3-1))
#define MASK4_ON 	Custom_Client_Data.stCustomClientInfo.masks|= (0x01<<(4-1))
#define MASK5_ON 	Custom_Client_Data.stCustomClientInfo.masks|= (0x01<<(5-1))
#define MASK6_ON 	Custom_Client_Data.stCustomClientInfo.masks|= (0x01<<(6-1))

#define MASK1_OFF	Custom_Client_Data.stCustomClientInfo.masks&= ~(0x01<<(1-1))
#define MASK2_OFF	Custom_Client_Data.stCustomClientInfo.masks&= ~(0x01<<(2-1))
#define MASK3_OFF	Custom_Client_Data.stCustomClientInfo.masks&= ~(0x01<<(3-1))
#define MASK4_OFF	Custom_Client_Data.stCustomClientInfo.masks&= ~(0x01<<(4-1))
#define MASK5_OFF	Custom_Client_Data.stCustomClientInfo.masks&= ~(0x01<<(5-1))
#define MASK6_OFF	Custom_Client_Data.stCustomClientInfo.masks&= ~(0x01<<(6-1))

/*------------------------哨兵通信----------------------------*/
#define Danger_Level_Low     70;
#define Danger_Level_High   200;

#define Robo_Data_Cmd_ID   0x0200
#define Client_Data_Cmd_ID 0x0110

#define Robo_Data_Length       7
#define Client_Data_Length    19
#define Client_Graphic_Length 67

int show_float(float x);
void Rc_RsysProtocol(USHORT16 rxSize);
void MonitorDataDeal(USHORT16 usCmdID);
void Send_Custom_Data(void);
void Send_Missile_Data(void);
void sendPictureToClient(void* picture, u16 pictueNum);
void sendDartToClient(SINT32 data1);
void sendDataToClient(SINT32 data1, SINT32 data2, SINT32 data3, SINT32 data4, SINT32 data5);
void sendFloatToClient(float data1, float data2, float data3, float data4, float data5);
void sendSEVENFloatToClient(float data1, float data2, float data3, float data4, float data5, float data6, float data7);
void pictureOperation(graphic_data_struct_t* pGraphicData, u8* graphicName, u32 operateTpye, u32 graphicTpye,
						          u32 layer, u32 color, u32 startAngle, u32 endAngle, u32 width, u32 startX, u32 startY,
						          u32 radius, u32 endX, u32 endY);
void Send_Sentinel_Data(void); 
void sendRectangleToClient(void);
void sendLobToClient(void);
void Press_Y_N(void);
void Rc_RsysProtocol_U6(USHORT16 rxSize);
void MonitorDataDeal_U6(USHORT16 usCmdID);

extern float hp_print;
extern bool testf; 
#endif
