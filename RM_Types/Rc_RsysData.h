#ifndef __RC_RSYSDATA_H
#define __RC_RSYSDATA_H
#include "rm_types_my.h"
#define Custom_Robo_Length 113
#define Custom_Client_Length 13
#define Custom_Graphic_length 61

/********************************************************************************************************/
/** 比赛状态数据：0x0001    发送频率：1Hz
 * @brief 	提供比赛剩余时间
 */
typedef __packed struct
{
	UCHAR8 game_type : 4;		// 1, RM主赛 2, RM单项赛 3，ICRA 4.3V3 5.1V1
	UCHAR8 game_progress : 4;	// 0 未开始比赛 1 准备阶段 2自检 3，5s倒计时 4，对战中 5，比赛结算中
	USHORT16 stage_remain_time; // 当前阶段剩余时间，s
	DB64 SyncTimeStamp;			// UNIX 时间，当机器人正确连接到裁判系统的 NTP 服务器后生效
} ext_game_status_t;
typedef union
{
	UCHAR8 ucBuf[11];
	ext_game_status_t stGameStatusInfo;
} UN_GameStatus;

/** 比赛结果数据：0x0002    发送频率：比赛结束后发送 */
typedef __packed struct
{
	UCHAR8 winner;
} ext_game_result_t;
typedef union
{
	UCHAR8 ucBuf[1];
	ext_game_result_t stGameResultInfo;
} UN_GameResult;

/** 机器人血量数据：0x0003   发送频率：1Hz
 * @brief	提供各队机器人血量数据
 */
typedef __packed struct
{
	USHORT16 red_1_robot_HP;
	USHORT16 red_2_robot_HP;
	USHORT16 red_3_robot_HP;
	USHORT16 red_4_robot_HP;
	USHORT16 red_5_robot_HP;
	USHORT16 red_7_robot_HP;
	USHORT16 red_outpost_HP; // 前哨站
	USHORT16 red_base_HP;	 // 基地
	USHORT16 blue_1_robot_HP;
	USHORT16 blue_2_robot_HP;
	USHORT16 blue_3_robot_HP;
	USHORT16 blue_4_robot_HP;
	USHORT16 blue_5_robot_HP;
	USHORT16 blue_7_robot_HP;
	USHORT16 blue_outpost_HP;
	USHORT16 blue_base_HP;
} ext_game_robot_HP_t; // 0x0003 血量数据 32字节
typedef union
{
	UCHAR8 ucBuf[32];
	ext_game_robot_HP_t stGameRobotHPInfo;
} UN_GameRobotHP;

/** 飞镖发送状态：0x0004    发送频率：飞镖发射后发送    发送范围：所有机器人
 *  @brief	敌方飞镖发射状态//在新的裁判系统串口协议里面没看到？//2023未看到
 */
typedef __packed struct
{
	uint8_t dart_belong;
	uint16_t Stage_remaining_time;
} ext_dart_status_t;
typedef union
{
	UCHAR8 ucBuf[3];
	ext_dart_status_t stDartStatusInfo;
} UN_DartStatus;

/** 人工智能挑战赛加成与惩罚区状态：0x0005   发送频率：1Hz周期发送    发送范围：所有机器人 */
typedef __packed struct
{
	UCHAR8 F1_zone_status : 1;
	UCHAR8 F1_zone_buff_debuff_status : 3;
	UCHAR8 F2_zone_status : 1;
	UCHAR8 F2_zone_buff_debuff_status : 3;
	UCHAR8 F3_zone_status : 1;
	UCHAR8 F3_zone_buff_debuff_status : 3;
	UCHAR8 F4_zone_status : 1;
	UCHAR8 F4_zone_buff_debuff_status : 3;
	UCHAR8 F5_zone_status : 1;
	UCHAR8 F5_zone_buff_debuff_status : 3;
	UCHAR8 F6_zone_status : 1;
	UCHAR8 F6_zone_buff_debuff_status : 3;

	USHORT16 red1_bullet_left;
	USHORT16 red2_bullet_left;
	USHORT16 blue1_bullet_left;
	USHORT16 blue2_bullet_left;
	UCHAR8 lurk_mode;
	UCHAR8 res;
} ext_ICRA_buff_debuff_zone_and_lurk_status_t;
typedef union
{
	UCHAR8 ucBuf[13];
	ext_ICRA_buff_debuff_zone_and_lurk_status_t stICRABuffDebuffZoneStatusInfo;
} UN_ICRABuffDebuffZoneStatus;

/** 场地事件数据：0x0101    发送频率：事件改变后发送 */
typedef __packed struct
{
	UINT32 event_type;
} ext_event_data_t;
typedef union
{
	UCHAR8 ucBuf[4];
	ext_event_data_t stEventDataInfo;
} UN_EventData;

/** 补给站动作标识：0x0102   发送频率：动作改变后发送    发生范围：己方机器人
 * @brief  补给站供弹信息
 */
typedef __packed struct
{
	UCHAR8 supply_projectile_id;
	UCHAR8 supply_robot_id;
	UCHAR8 supply_projectile_step;
	UCHAR8 supply_projectile_num;
} ext_supply_projectile_action_t;
typedef union
{
	UCHAR8 ucBuf[4];
	ext_supply_projectile_action_t stSupplyProjectileActionInfo;
} UN_SupplyProjectileAction;

/** 裁判警告信息：0x0104    发送频率：警告发生后发送*/
typedef __packed struct
{
	UCHAR8 level;
	UCHAR8 foul_robot_id;
} ext_referee_warning_t;
typedef union
{
	UCHAR8 ucBuf[2];
	ext_referee_warning_t stRefereeWarningInfo;
} UN_RefereeWarning;

/** 飞镖发射口倒计时：0x0105  发射频率：1Hz    发送范围：己方机器人 */
typedef __packed struct
{
	UCHAR8 dart_remaining_time;
} ext_dart_remaining_time_t;
typedef union
{
	UCHAR8 ucBuf[1];
	ext_dart_remaining_time_t stDartRemainingTimeInfo;
} UN_DartRemainingTime;

/** 比赛机器人状态：0x0201   发送频率：10Hz */
// typedef __packed struct
//{
//    UCHAR8 robot_id;         //本机器人 ID
//    UCHAR8 robot_level;      //机器人等级
//    USHORT16 remain_HP;      //机器人剩余血量
//    USHORT16 max_HP;         //机器人上限血量
//    USHORT16 shooter_id1_17mm_cooling_rate;   //机器人 1 号 17mm 枪口每秒冷却值
//    USHORT16 shooter_id1_17mm_cooling_limit;  //机器人 1 号 17mm 枪口热量上限
//    USHORT16 shooter_id1_17mm_speed_limit;    //机器人 1 号 17mm 枪口上限速度 单位 m/s
//
//    USHORT16 shooter_id2_17mm_cooling_rate;   //机器人 2 号 17mm 枪口每秒冷却值
//    USHORT16 shooter_id2_17mm_cooling_limit;  //机器人 2 号 17mm 枪口热量上限
//    USHORT16 shooter_id2_17mm_speed_limit;    //机器人 2 号 17mm 枪口上限速度 单位 m/s
//
//	 USHORT16 shooter_id1_42mm_cooling_rate;   //机器人 42mm 枪口每秒冷却值
//	 USHORT16 shooter_id1_42mm_cooling_limit;  //机器人 42mm 枪口热量上限
//	 USHORT16 shooter_id1_42mm_speed_limit;    //机器人 42mm 枪口上限速度 单位 m/s
//
//    USHORT16 chassis_power_limit;              //机器人底盘功率限制上限
//    UCHAR8 mains_power_gimbal_output : 1;   //gimbal口输出:1有输出0无输出
//    UCHAR8 mains_power_chassis_output : 1; //chassis口输出:1有输出0无输出
//    UCHAR8 mains_power_shooter_output : 1; //shooter口输出:1有输出0无输出
// } ext_game_robot_state_t;
// typedef union
//{
//	UCHAR8 ucBuf[29];
//	ext_game_robot_state_t stGameRobotStatusInfo;
// } UN_GameRobotStatus;

typedef __packed struct
{
	UCHAR8 robot_id;					   // 本机器人 ID
	UCHAR8 robot_level;					   // 机器人等级
	USHORT16 remain_HP;					   // 机器人剩余血量
	USHORT16 max_HP;					   // 机器人上限血量
	USHORT16 shooter_barrel_cooling_value; // 机器人枪口每秒冷却值
	USHORT16 shooter_barrel_heat_limit;	   // 机器人枪口热量上限
	USHORT16 chassis_power_limit;		   // 机器人底盘功率限制上限
	UCHAR8 mains_power_gimbal_output : 1;  // gimbal口输出:1有输出0无输出
	UCHAR8 mains_power_chassis_output : 1; // chassis口输出:1有输出0无输出
	UCHAR8 mains_power_shooter_output : 1; // shooter口输出:1有输出0无输出
} ext_game_robot_state_t;
typedef union
{
	UCHAR8 ucBuf[13];
	ext_game_robot_state_t stGameRobotStatusInfo;
} UN_GameRobotStatus;

/** 实时功率热量数据：0x0202  发送频率：50Hz */
typedef __packed struct
{
	USHORT16 chassis_volt;					// 底盘电压
	USHORT16 chassis_current;				// 底盘电流
	FP32 chassis_power;						// 底盘输出功率
	USHORT16 chassis_power_buffer;			// 底盘功率缓冲
	USHORT16 shooter_id1_17mm_cooling_heat; // 1 号 17mm 枪口热量
	USHORT16 shooter_id2_17mm_cooling_heat; // 2 号 17mm 枪口热量
	USHORT16 shooter_id1_42mm_cooling_heat; // 42mm枪口热量
} ext_power_heat_data_t;
typedef union
{
	UCHAR8 ucBuf[16];
	ext_power_heat_data_t stPowerHeatDataInfo;
} UN_PowerHeatData;

/** 机器人位置：0x0203     发送频率：10Hz */
typedef __packed struct
{
	FP32 x;	  // 位置坐标x,单位m
	FP32 y;	  // 位置坐标y,单位m
	FP32 yaw; // 位置枪口，单位度,正北为0度
} ext_game_robot_pos_t;
typedef union
{
	UCHAR8 ucBuf[12];
	ext_game_robot_pos_t stGameRobotPosInfo;
} UN_GameRobotPos;

/** 机器人增益：0x0204     发送频率：1Hz */
typedef __packed struct
{
	UCHAR8 power_rune_buff;
} ext_buff_t;
typedef union
{
	UCHAR8 ucBuf[1];
	ext_buff_t stBuffInfo;
} UN_Buff;

/** 空中机器人能量状态：0x0205     发送频率：10Hz */
// 无人机裁判系统通讯数据格式定义
typedef __packed struct
{
	UCHAR8 airforce_status; // 飞机状态（0 为正在冷却，1 为冷却完毕，2 为正在空中支援）
	UCHAR8 time_remain;		// 此状态的剩余时间（单位为：秒，向下取整，即冷却时间剩余 1.9 秒时，此值为 1）若冷却时间为 0，但未呼叫空中支援，则该值为 0
} ext_aerial_robot_energy_t;
typedef union
{
	UCHAR8 ucBuf[2];
	ext_aerial_robot_energy_t stAerialRobotEnergyInfo;
} UN_AerialRobotEnergy;

/** 伤害状态：0x0206          发送频率：伤害发生后发送 */
typedef __packed struct
{
	UCHAR8 armor_id : 4;
	UCHAR8 hurt_type : 4;
} ext_robot_hurt_t;
typedef union
{
	UCHAR8 ucBuf[1];
	ext_robot_hurt_t stRobotHurtInfo;
} UN_RobotHurt;

/** 实时射击信息：0x0207    发送频率：射击后发送 */
typedef __packed struct
{
	UCHAR8 bullet_type; // 弹丸种类
	UCHAR8 shooter_id;	// 发射机构 ID
	UCHAR8 bullet_freq; // 发射频率
	FP32 bullet_speed;	// 弹丸出射速度
} ext_shoot_data_t;
typedef union
{
	UCHAR8 ucBuf[7];
	ext_shoot_data_t stShootDataInfo;
} UN_ShootData;

/** 子弹剩余发射数：0x0208   发送频率：10Hz周期发送   发送范围：所有机器人发送 */
typedef __packed struct
{
	USHORT16 bullet_remaining_num_17mm; // 17mm 子弹剩余发射数目
	USHORT16 bullet_remaining_num_42mm; // 42mm 子弹剩余发射数目
	USHORT16 coin_remaining_num;		// 剩余金币数量
	USHORT16 projectile_allowance_fortress; //堡垒增益点提供的储备17mm弹丸允许发弹量；
} ext_bullet_remaining_t;				// 0x0208 子弹剩余 2字节
typedef union
{
	UCHAR8 ucBuf[8];
	ext_bullet_remaining_t stBulletRemainingInfo;
} UN_BulletRemaining;

/** 机器人RFID状态：0x0209     发送频率：1Hz    发送范围：单一机器人 */
typedef __packed struct
{
	UINT32 rfid_status;
} ext_rfid_status_t;
typedef union
{
	UCHAR8 ucBuf[4];
	ext_rfid_status_t stRFIDStatusInfo;
} UN_RFIDStatus;

/** 飞镖机器人客户端指令数据：0x020A  发送频率：10Hz   发送范围：单一机器人 */
typedef __packed struct
{
	UCHAR8 dart_launch_opening_status;
	UCHAR8 dart_attack_target;
	USHORT16 target_change_time;
	USHORT16 operate_launch_cmd_time;
} ext_dart_client_cmd_t;
typedef union
{
	UCHAR8 ucBuf[6];
	ext_dart_client_cmd_t stDartClientCmdInfo;
} UN_DartClientCmd;

/** 交互数据接收信息：0x0301 */
#define RobotInteractiveDataID 0x0301
typedef __packed struct // 机器人交互数据格式
{
	USHORT16 data_cmd_id;
	USHORT16 send_ID;
	USHORT16 receiver_ID;
	UCHAR8 data[Custom_Robo_Length];
} ext_robot_interactive_data_t;
typedef union
{
	UCHAR8 ucBuf[Custom_Robo_Length + 6];
	ext_robot_interactive_data_t stCustomInfo;
} UN_Custom_Robo;

typedef __packed struct // 客户端自定义数据,新的串口协议没看到，是以前的？
{
	USHORT16 data_cmd_id;
	USHORT16 send_ID;
	USHORT16 receiver_ID;
	FP32 data1;
	FP32 data2;
	FP32 data3;
	UCHAR8 masks;
} client_custom_data_t;

typedef union
{
	UCHAR8 ucBuf[Custom_Client_Length + 6];
	client_custom_data_t stCustomClientInfo;
} UN_Custom_Client;

// 交互数据接收信息： 0x0301。
typedef __packed struct
{
	USHORT16 data_cmd_id;
	USHORT16 sender_ID;
	USHORT16 receiver_ID;
} ext_student_interactive_header_data_t;

// 1.交互数据 机器人间通信： 0x0301。
typedef __packed struct
{
	UCHAR8 m_SendData;
} robot_interactive_data_t;

// 2.客户端删除图形 机器人间通信： 0x0301。
typedef __packed struct
{
	UCHAR8 operate_tpye;
	UCHAR8 layer;
} ext_client_custom_graphic_delete_t;

// 图形数据
typedef __packed struct
{
	UCHAR8 graphic_name[3];
	UINT32 operate_tpye : 3;
	UINT32 graphic_tpye : 3;
	UINT32 layer : 4;
	UINT32 color : 4;
	UINT32 start_angle : 9;
	UINT32 end_angle : 9;
	UINT32 width : 10;
	UINT32 start_x : 11;
	UINT32 start_y : 11;
	UINT32 radius : 10;
	UINT32 end_x : 11;
	UINT32 end_y : 11;
} graphic_data_struct_t;

// 3.客户端绘制一个图形 机器人间通信： 0x0301。
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;

// 4.客户端绘制二个图形 机器人间通信： 0x0301。
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;

// 5.客户端绘制五个图形 机器人间通信： 0x0301。
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;

// 6.客户端绘制七个图形 机器人间通信： 0x0301。
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;

// 7.客户端绘制字符 机器人间通信： 0x0301。
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
	UCHAR8 data[30];
} ext_client_custom_character_t;

// 接收其他机器人通讯数据的结构体
typedef __packed struct
{
	UCHAR8 Alarm_OverSpeed;
	UCHAR8 remain_num;
	UCHAR8 feedback;
} ext_missile_data_t; // 接收飞镖

typedef __packed struct
{
	UCHAR8 feedback;
	UCHAR8 DATA2;
} ext_sentinel_data_t; // 接收哨兵

typedef __packed struct
{
	UCHAR8 capa_voltage;
	UCHAR8 remain_bullet;
} ext_ground_data_t; // 接收地面部队

// 通过遥控器发送的键鼠遥控数据将同步通过图传链路发送给对应机器人
// 命令码ID:0x0304
typedef __packed struct
{
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	int8_t left_button_down;
	int8_t right_button_down;
	uint16_t keyboard_value;
	uint16_t reserved;
} remote_control_t;

typedef __packed struct
{
//    uint8_t soft_1;
//    uint8_t soft_2;
    uint16_t ch_0; //11
    uint16_t ch_1; 
    uint16_t ch_2;
    uint16_t ch_3;
    uint8_t mode_sw; //2
    uint8_t go_home; //1
    uint8_t fn; //1
    uint8_t button; //1
    uint16_t wheel; //11
    uint8_t shutter; //1
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t left_button_down; //2
    uint8_t right_button_down; //2
    uint8_t middle_button_down; //2
    uint16_t keyboard_value; 
//    uint16_t crcl6;
}remote_control_keyboard_mouset_t;


#endif
