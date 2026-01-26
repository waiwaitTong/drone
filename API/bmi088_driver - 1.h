#ifndef __BMI088_H__
#define __BMI088_H__

//#include "spi.h"
//#include "delay.h"
//#include "math.h"
#include "global_declare.h"

#define spi_read_write_byte 	spi1_read_write_byte

#define  GYRO_CS_H    GPIO_SetBits(Open_SPIx_CSB2_PORT, Open_SPIx_CSB2_PIN)//GPIOB   GPIO_Pin_12
#define  GYRO_CS_L    GPIO_ResetBits(Open_SPIx_CSB2_PORT, Open_SPIx_CSB2_PIN)//GPIOB   GPIO_Pin_12

#define  ACC_CS_H    GPIO_SetBits(Open_SPIx_CSB1_PORT, Open_SPIx_CSB1_PIN)//GPIOC  GPIO_Pin_8
#define  ACC_CS_L    GPIO_ResetBits(Open_SPIx_CSB1_PORT, Open_SPIx_CSB1_PIN)//GPIOC  GPIO_Pin_8

//Accelerater_Reg

#define	ACC_ID				0X00
#define	ACC_ERR				0X02
#define ACC_STATUS		0X03
#define	ACC_X_LSB			0X12
#define	ACC_X_MSB			0X13
#define	ACC_Y_LSB 		0X14
#define	ACC_Y_MSB 		0X15
#define	ACC_Z_LSB 		0X16
#define	ACC_Z_MSB 		0X17
#define ACC_RANGE			0X41
#define	TEMP_MSB			0X22
#define	TEMP_LSB 			0X23
#define ACC_CONF			0X40
#define	ACC_PWR_CTRL	0X7D
#define ACC_PWR_CONF	0X7C
#define ACC_SOFT_R		0X7E


//Gyro_Reg

#define	GYRO_ID				0X00
#define	GYRO_X_LSB		0X02
#define	GYRO_X_MSB		0X03
#define	GYRO_Y_LSB		0X04
#define	GYRO_Y_MSB		0X05
#define	GYRO_Z_LSB		0X06
#define	GYRO_Z_MSB		0X07
#define	GYRO_RANGE		0X0F
#define	GYRO_BANDW		0X10
#define GYRO_POWER		0X11
#define	GYRO_SOFT_R		0X14

#define CALI_NUM			5000

#define FAIL          0
#define SUCCESS       1

#define NOP			__asm{NOP;}

#define OFF_FREQ			1500

#define Cali_Rol_Coe 1
#define Cali_Pit_Coe 1
#define Cali_Yaw_Coe 1.0021f	

#ifndef PI
  #define PI               3.14159265358979f
#endif

enum {X = 0, Y = 1, Z = 2, VEC_XYZ};
enum {A_X = 0, A_Y, A_Z, G_X, G_Y, G_Z, TEM, MPU_ITEMS};
typedef enum {INIT = 0, NORMAL = 1, CALIBRATION = 2}IMU_MODE;

typedef struct
{
	float center_pos_cm[VEC_XYZ];
	float gyro_rad[VEC_XYZ];
	float gyro_rad_old[VEC_XYZ];
	float gyro_rad_acc[VEC_XYZ];
	float linear_acc[VEC_XYZ];
}_center_pos_st;

typedef struct 
{
	u8 acc_CALIBRATE;
	u8 gyr_CALIBRATE;
	u8 acc_z_auto_CALIBRATE;
	u8 temp_CALIBRATE;
	u8 sensor_ok;
	u8 motionless;
	s16 Acc_Original[VEC_XYZ];	//加速度原始数据
	s16 Gyro_Original[VEC_XYZ];	//角度原始数据
	s16 Tempreature;
	float Tempreature_C;
	
	float Acc[VEC_XYZ];       //模拟量
	float Acc_cmss[VEC_XYZ];  //单位是cm/s2
	float Gyro[VEC_XYZ];      //模拟量
	float Gyro_deg[VEC_XYZ];  //单位是角度/s
	float Gyro_rad[VEC_XYZ];  //单位是弧度/s
	
	float acc_coe[VEC_XYZ];   //加速度计刻度因数
	float acc_offset[VEC_XYZ];//加速度计零偏
	float gyro_offset[VEC_XYZ];	
}_sensor_st;

typedef struct
{
	float preout;
	float out;
	float in;
	float off_freq;
	float samp_tim;
}ST_LPF;


typedef struct
{
	float q0;
	float q1;
	float q2;
	float q3;
	
	float gkp;
	float gki;

	float x_vec[VEC_XYZ];
	float y_vec[VEC_XYZ];
	float z_vec[VEC_XYZ];

	float a_acc[VEC_XYZ];
	float gacc_deadzone[VEC_XYZ];
	float gra_acc[VEC_XYZ];
	
	float rol;
	float pit;
	float yaw;
} _imu_st ;


bool bmi088_init(void);
UCHAR8 BMI088_Read_Gyro_Data(UCHAR8 reg_id);
void BMI088_Write_Gyro_Data(UCHAR8 reg_id,UCHAR8 data);
UCHAR8 BMI088_Read_Acc_Data(UCHAR8 reg_id);
void BMI088_Write_Acc_Data(UCHAR8 reg_id,UCHAR8 data);
void Sensor_Data_Prepare(void);
void LpFilter(ST_LPF* lpf);
float invSqrt(float x);



extern SSHORT16 Real_Temp;
extern FP32 Gyro_X_Ori;
extern FP32 Gyro_Y_Ori;
extern FP32 Gyro_Z_Ori;
extern IMU_MODE imu_mode;

extern float gyro_offset_Pit;
extern float gyro_offset_Yaw;
extern float gyro_offset_Rol;

extern float Gyro_X_Real;
extern float Gyro_Y_Real;
extern float Gyro_Z_Real;

extern float Acc_X_Real;
extern float Acc_Y_Real;
extern float Acc_Z_Real;

extern float Gyro_X_Speed;
extern float Gyro_Y_Speed;
extern float Gyro_Z_Speed;





#endif






