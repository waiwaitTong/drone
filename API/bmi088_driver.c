#include "bmi088_driver.h"
#include "math.h"
extern IMU_MODE imu_mode;

Filter6axisTypeDef Filters;
_sensor_st sensor;
u8 butt_flag = 0;

ST_LPF gyro_x = {0, 0, 0, 150, 0.00025f}; // 陀螺仪
ST_LPF gyro_y = {0, 0, 0, 150, 0.00025f};
ST_LPF gyro_z = {0, 0, 0, 150, 0.00025f};

ST_LPF acc_x = {0, 0, 0, 500, 0.00025f}; // 加速度
ST_LPF acc_y = {0, 0, 0, 500, 0.00025f};
ST_LPF acc_z = {0, 0, 0, 500, 0.00025f};

ST_LPF Gyro_x = {0, 0, 0, 10, 0.0005f};
ST_LPF Gyro_y = {0, 0, 0, 10, 0.0005f};
ST_LPF Gyro_z = {0, 0, 0, 10, 0.0005f};

FP32 Cali_Cnt = CALI_NUM;
FP32 Real_Temp;
FP32 Acc_X_Ori;
FP32 Acc_Y_Ori;
FP32 Acc_Z_Ori;
FP32 Gyro_X_Ori;
FP32 Gyro_Y_Ori;
FP32 Gyro_Z_Ori;

FP32 Acc_X_Real; // 单位为mg
FP32 Acc_Y_Real;
FP32 Acc_Z_Real;

FP32 Gyro_X_Real; // 单位为弧度每秒
FP32 Gyro_Y_Real;
FP32 Gyro_Z_Real;

/*以下为零飘标定值*/
FP32 Acc_X_Offset = 0;
FP32 Acc_Y_Offset = 0;
FP32 Acc_Z_Offset = 0;



//FP32	Gyro_X_Offset =0.0920979232f;//这个得实测，每个陀螺仪有细微的差别
//FP32	Gyro_Y_Offset =0.116354682f;
//FP32    Gyro_Z_Offset =0.130834997f;	

FP32		Gyro_X_Offset = -0.170390576f;//这个得实测，每个陀螺仪有细微的差别
FP32		Gyro_Y_Offset = -0.146174371f;
FP32	    Gyro_Z_Offset = 0.127867877f;		

// FP32		 Gyro_X_Offset =0.101077288;
// FP32	     Gyro_Y_Offset =0.0723787844;
// FP32         Gyro_Z_Offset =-0.215208098;

// FP32 Status_offset[3][CALI_NUM] = {0};  //1 X;2 Y;3 Z.
// FP32 Num_offset[CALI_NUM] = {0};
int Const_offset = CALI_NUM;
int flag_offset = 0;
FP32 testvalue[3] = {0};

FP32 SEE = 0; // Z_OFFSET实时标定的值

#define STM32_FLASH_BASE 0x08000000 // STM32 FLASH的起始地址

// u32 STMFLASH_ReadWord(u32 faddr)
//{
//	return *(vu32*)faddr;
// }

// 获取某个地址所在的flash扇区
// addr:flash地址
// 返回值:0~11,即addr所在的扇区
// uint16_t STMFLASH_GetFlashSector(u32 addr)
//{
//	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
//	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
//	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
//	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
//	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
//	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
//	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
//	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
//	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
//	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
//	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10;
//	return FLASH_Sector_11;
// }

FP32 Test_Flash_Ori = 1.1;
FP32 Test_Flash_Write[2] = {1.1, 2.2};

FP32 Test_Flash_Get[2] = {0, 0};

/*--------------------------------------
任务功能：写入flash
备注：原自带flash函数无法使用
--------------------------------------*/
// void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)
//{
//   FLASH_Status status = FLASH_COMPLETE;
//	u32 addrx=0;
//	u32 endaddr=0;
//   if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//非法地址
//	FLASH_Unlock();									//解锁
//   FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存

//	addrx=WriteAddr;				//写入的起始地址
//	endaddr=WriteAddr+NumToWrite*4;	//写入的结束地址
//	if(addrx<0X1FFF0000)			//只有主存储区,才需要执行擦除操作!!
//	{
//		while(addrx<endaddr)		//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
//		{
//			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//有非0XFFFFFFFF的地方,要擦除这个扇区
//			{
//				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V之间!!
//				if(status!=FLASH_COMPLETE)break;	//发生错误了
//			}else addrx+=4;
//		}
//	}
//	if(status==FLASH_COMPLETE)
//	{
//		while(WriteAddr<endaddr)//写数据
//		{
//			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)//写入数据
//			{
//				break;	//写入异常
//			}
//			WriteAddr+=4;
//			pBuffer++;
//		}
//	}
//  FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
//	FLASH_Lock();//上锁
//}

UCHAR8 BMI088_Read_Gyro_Data(UCHAR8 reg_id)
{
    UCHAR8 rec;

    GYRO_CS_L;
    rec = (UCHAR8)(0x00ff & spi1_read_write_byte((USHORT16)((reg_id << 8 | 0x00) | 0x8000))); // 0b1000000000000000)));
    GYRO_CS_H;
    NOP;

    return rec;
}

void BMI088_Write_Gyro_Data(UCHAR8 reg_id, UCHAR8 data)
{
    GYRO_CS_L;
    spi1_read_write_byte((USHORT16)((reg_id << 8 | data) & 0x7fff)); // 0b0111111111111111));
    GYRO_CS_H;
    NOP;
}

UCHAR8 BMI088_Read_Acc_Data(UCHAR8 reg_id)
{
    UCHAR8 rec = 0;

    ACC_CS_L;
    spi1_read_write_byte((USHORT16)((reg_id << 8 | 0xff) | 0x8000)); // 0b1000000000000000));				//Dummy_Byte
    rec = (UCHAR8)(spi1_read_write_byte(0x0000) >> 8) & 0xff;
    ACC_CS_H;
    NOP;

    return rec;
}

void BMI088_Write_Acc_Data(UCHAR8 reg_id, UCHAR8 data)
{
    ACC_CS_L;
    spi1_read_write_byte((USHORT16)((reg_id << 8 | data) & 0x7fff)); // & 0b0111111111111111));
    ACC_CS_H;
    NOP;
}

UINT32 Acc_Range_Coe;

UCHAR8 Acc_ID, Gyro_ID;
int GetStill = 0;
bool bmi088_init(void) // IMU初始化
{
    BMI088_Read_Acc_Data(ACC_ID);
    BMI088_Read_Gyro_Data(GYRO_ID);

    delay_ms(100);

    BMI088_Write_Acc_Data(ACC_PWR_CONF, 0x00);
    BMI088_Write_Acc_Data(ACC_PWR_CTRL, 0x04);

    BMI088_Read_Acc_Data(ACC_ID);
    BMI088_Read_Gyro_Data(GYRO_ID);

    // 初始化时要求启动Acc为Normal模式，需要写PWR寄存器

    Acc_ID = BMI088_Read_Acc_Data(ACC_ID);
    Gyro_ID = BMI088_Read_Gyro_Data(GYRO_ID);

    if (Acc_ID != 0x1e || Gyro_ID != 0x0f) // ID_Wrong
    {
        BMI088_Write_Acc_Data(ACC_SOFT_R, 0XB6);
        BMI088_Write_Gyro_Data(GYRO_SOFT_R, 0XB6);
        delay_ms(50);
        return TRUE;
    }

    if ((BMI088_Read_Acc_Data(ACC_ERR) & 0x1D /*0b00011101*/) != 0) // Acc_Err_Ocur
    {
        BMI088_Write_Acc_Data(ACC_SOFT_R, 0XB6);
        BMI088_Write_Gyro_Data(GYRO_SOFT_R, 0XB6);
        delay_ms(50);
        return TRUE;
    }

    // Acc配置
    // 配置ODR，Filter
    BMI088_Write_Acc_Data(ACC_CONF, 0XAC); // 0b10101100);										//配置加速度计输出速率1600Hz，Normal输出模式
    delay_ms(10);
    // 配置测量范围
    BMI088_Write_Acc_Data(ACC_RANGE, 0X01); // 0b00000001);								  //配置加速度计输出范围+-6g
    delay_ms(10);
    // 配置电源属性
    BMI088_Write_Acc_Data(ACC_PWR_CONF, 0x00); // 配置进入活动模式
    delay_ms(10);
    // 配置电源开关
    BMI088_Write_Acc_Data(ACC_PWR_CTRL, 0x04); // 打开加速度计电源
    delay_ms(10);
    // 计算加速度计量程系数
    Acc_Range_Coe = pow(2, (BMI088_Read_Acc_Data(ACC_RANGE) + 1)) + 2;
    delay_ms(10);

    // Gyro配置
    // 配置测量范围
    BMI088_Write_Gyro_Data(GYRO_RANGE, 0X01); // 配置陀螺仪量程+-1000°/s
    delay_ms(10);
    // 配置低通滤波带宽
    BMI088_Write_Gyro_Data(GYRO_BANDW, 0X01); // 配置输出速率2000Hz，带宽532Hz
    delay_ms(10);
    // 配置电源模式
    BMI088_Write_Gyro_Data(GYRO_POWER, 0x00); // 正常模式
    delay_ms(10);

    delay_ms(100);
    return FALSE;
}

void GetValue(void)
{
    UCHAR8 Acc_X_MSB;
    UCHAR8 Acc_X_LSB;
    UCHAR8 Acc_Y_MSB;
    UCHAR8 Acc_Y_LSB;
    UCHAR8 Acc_Z_MSB;
    UCHAR8 Acc_Z_LSB;

    UCHAR8 Gyro_X_MSB;
    UCHAR8 Gyro_X_LSB;
    UCHAR8 Gyro_Y_MSB;
    UCHAR8 Gyro_Y_LSB;
    UCHAR8 Gyro_Z_MSB;
    UCHAR8 Gyro_Z_LSB;

    UCHAR8 Temp_MSB;
    UCHAR8 Temp_LSB;
    USHORT16 Temp_Ori;

    // 原始数据获取

    // 判断数据状态？Acc_Data_Rdy_Reg   ACC_STATUS
    Acc_X_MSB = BMI088_Read_Acc_Data(ACC_X_MSB);
    Acc_X_LSB = BMI088_Read_Acc_Data(ACC_X_LSB);
    Acc_Y_MSB = BMI088_Read_Acc_Data(ACC_Y_MSB);
    Acc_Y_LSB = BMI088_Read_Acc_Data(ACC_Y_LSB);
    Acc_Z_MSB = BMI088_Read_Acc_Data(ACC_Z_MSB);
    Acc_Z_LSB = BMI088_Read_Acc_Data(ACC_Z_LSB);

    Gyro_X_MSB = BMI088_Read_Gyro_Data(GYRO_X_MSB);
    Gyro_X_LSB = BMI088_Read_Gyro_Data(GYRO_X_LSB);
    Gyro_Y_MSB = BMI088_Read_Gyro_Data(GYRO_Y_MSB);
    Gyro_Y_LSB = BMI088_Read_Gyro_Data(GYRO_Y_LSB);
    Gyro_Z_MSB = BMI088_Read_Gyro_Data(GYRO_Z_MSB);
    Gyro_Z_LSB = BMI088_Read_Gyro_Data(GYRO_Z_LSB);

    Acc_X_Ori = ((SSHORT16)(Acc_X_MSB << 8 | Acc_X_LSB)) / 32767.0f * 1000.0f * Acc_Range_Coe; // Range为加速度计设定的测量范围，单位m*g

    Acc_Y_Ori = ((SSHORT16)(Acc_Y_MSB << 8 | Acc_Y_LSB)) / 32767.0f * 1000.0f * Acc_Range_Coe; // Range = 6g;
    Acc_Z_Ori = ((SSHORT16)(Acc_Z_MSB << 8 | Acc_Z_LSB)) / 32767.0f * 1000.0f * Acc_Range_Coe;

    Gyro_X_Ori = ((SSHORT16)(Gyro_X_MSB << 8 | Gyro_X_LSB)) / 32767.0f * 1000.0f; // 单位°/s
    Gyro_Y_Ori = ((SSHORT16)(Gyro_Y_MSB << 8 | Gyro_Y_LSB)) / 32767.0f * 1000.0f; // Range = 1000°/s
    Gyro_Z_Ori = ((SSHORT16)(Gyro_Z_MSB << 8 | Gyro_Z_LSB)) / 32767.0f * 1000.0f;

    // 易出现NaN
    Temp_MSB = BMI088_Read_Acc_Data(TEMP_MSB);
    Temp_LSB = BMI088_Read_Acc_Data(TEMP_LSB);

    Temp_Ori = (Temp_MSB * 8) + (Temp_LSB / 32);
    if (Temp_Ori > 1023)
        Real_Temp = (Temp_Ori - 2048) * 0.125 + 23;
    else
        Real_Temp = Temp_Ori * 0.125 + 23;
}

// 最小二乘法
FP32 leastSquareLinearFit(FP32 x[], FP32 y[], const int num)
{
    FP32 a = 0;
    // FP32 b = 0;
    FP32 sum_x = 0.0;
    FP32 sum_y = 0.0;
    FP32 sum_x2 = 0.0;
    FP32 sum_xy = 0.0;

    for (int i = 0; i < num; ++i)
    {
        sum_x2 += x[i] * x[i];
        sum_y += y[i];
        sum_x += x[i];
        sum_xy += x[i] * y[i];
    }

    FP32 c = num * sum_x2 - sum_x * sum_x;
    if (c == 0)
    {
        a = 10000;
    }
    else
    {
        a = (num * sum_xy - sum_x * sum_y) / (num * sum_x2 - sum_x * sum_x);
    }
    // b = (sum_x2*sum_y - sum_x*sum_xy)/(num*sum_x2-sum_x*sum_x);
    return a;
}

// 校准时蓝灯闪烁
int GetOffset(void)
{
    // 原始值获取中间参数
    UCHAR8 Gyro_X_MSB;
    UCHAR8 Gyro_X_LSB;
    UCHAR8 Gyro_Y_MSB;
    UCHAR8 Gyro_Y_LSB;
    UCHAR8 Gyro_Z_MSB;
    UCHAR8 Gyro_Z_LSB;
    // 读取BMI088陀螺仪的寄存器数据
    Gyro_X_MSB = BMI088_Read_Gyro_Data(GYRO_X_MSB);
    Gyro_X_LSB = BMI088_Read_Gyro_Data(GYRO_X_LSB);
    Gyro_Y_MSB = BMI088_Read_Gyro_Data(GYRO_Y_MSB);
    Gyro_Y_LSB = BMI088_Read_Gyro_Data(GYRO_Y_LSB);
    Gyro_Z_MSB = BMI088_Read_Gyro_Data(GYRO_Z_MSB);
    Gyro_Z_LSB = BMI088_Read_Gyro_Data(GYRO_Z_LSB);
    // 处理数据，获得陀螺仪原始值
    Gyro_X_Ori = ((SSHORT16)(Gyro_X_MSB << 8 | Gyro_X_LSB)) / 32767.0f * 1000.0f; // 单位°/s
    Gyro_Y_Ori = ((SSHORT16)(Gyro_Y_MSB << 8 | Gyro_Y_LSB)) / 32767.0f * 1000.0f; // Range = 1000°/s
    Gyro_Z_Ori = ((SSHORT16)(Gyro_Z_MSB << 8 | Gyro_Z_LSB)) / 32767.0f * 1000.0f;
    // 低通滤波
    Gyro_x.in = Gyro_X_Ori;
    LpFilter(&Gyro_x);
    Gyro_X_Ori = Gyro_x.out;

    Gyro_y.in = Gyro_Y_Ori;
    LpFilter(&Gyro_y);
    Gyro_Y_Ori = Gyro_y.out;

    Gyro_z.in = Gyro_Z_Ori;
    LpFilter(&Gyro_z);
    Gyro_Z_Ori = Gyro_z.out;
    // 倒计数执行
    if (Cali_Cnt)
    {
        //			Status_offset[0][CALI_NUM-Cali_Cnt] = Gyro_X_Ori;
        Gyro_X_Offset += Gyro_X_Ori;
        //			Status_offset[1][CALI_NUM-Cali_Cnt] = Gyro_Y_Ori;
        Gyro_Y_Offset += Gyro_Y_Ori;
        //			Status_offset[2][CALI_NUM-Cali_Cnt] = Gyro_Z_Ori;
        Gyro_Z_Offset += Gyro_Z_Ori;
        //			Num_offset[CALI_NUM-Cali_Cnt]       = CALI_NUM-Cali_Cnt;
        Cali_Cnt--;
    }
    else
    {
        for (int i = 0; i < 3; i++)
        {
            //				testvalue[i] = leastSquareLinearFit(Num_offset,Status_offset[i],CALI_NUM);			//获得原始数据的线性相关度
            if ((testvalue[i] > 0.00005l) || (testvalue[i] < -0.00005l)) // 根据相关度排除云控处于非静止状态标定
            {
                Cali_Cnt = CALI_NUM;
                Gyro_X_Offset = 0;
                Gyro_Y_Offset = 0;
                Gyro_Z_Offset = 0;
                return FAIL; // 0
            }
        }
        Gyro_X_Offset /= CALI_NUM;
        Gyro_Y_Offset /= CALI_NUM;
        Gyro_Z_Offset /= CALI_NUM;
        /*2024.1.14*/
        //		 Gyro_X_Offset =0.276311487f;
        //		 Gyro_Y_Offset = 0.0942971855f;
        //       Gyro_Z_Offset =0.556416333f;


        Cali_Cnt = CALI_NUM;
        return SUCCESS;
    }
    return FAIL;
}

void Manual_Fix(void)//手动标定
{
//         	Gyro_X_Offset += ;
//        	Gyro_Y_Offset += ;
    Gyro_Z_Offset +=0.000001* ((DR16_rec.stRC.Ch0-RC_CH_VALUE_OFFSET)?1:-1) ;
}



FP32 Limit_Zero(FP32 Target, FP32 limit)
{
    if (Target < limit && Target > -limit)
        return 0.0f;
    else
        return Target;
}
FP32 G_Cal;
bool Warning;

ST_LPF Gyro_X_Lpf = {0, 0, 0, OFF_FREQ, 0.00025f};
ST_LPF Gyro_Y_Lpf = {0, 0, 0, OFF_FREQ, 0.00025f};
ST_LPF Gyro_Z_Lpf = {0, 0, 0, OFF_FREQ, 0.00025f};

FP32 ORI_Gyrox = 0;
FP32 ORI_Gyroy = 0;
FP32 ORI_Gyroz = 0;

FP32 ORI_Accx = 0;
FP32 ORI_Accy = 0;
FP32 ORI_Accz = 0;

void Sensor_Data_Prepare(void) // IMU数据准备（坐标转换，滤波，矫正零偏）
{
    GetValue();
    // 矫正零偏
    Acc_X_Ori = Cali_Rol_CoeA * (Acc_X_Ori - Acc_X_Offset);
    Acc_Y_Ori = Cali_Rol_CoeA * (Acc_Y_Ori - Acc_Y_Offset);
    Acc_Z_Ori = Cali_Rol_CoeA * (Acc_Z_Ori - Acc_Z_Offset);

    Gyro_X_Ori = Cali_Rol_Coe * (Gyro_X_Ori - Gyro_X_Offset);
    Gyro_Y_Ori = Cali_Pit_Coe * (Gyro_Y_Ori - Gyro_Y_Offset);
    Gyro_Z_Ori = Cali_Yaw_Coe * (Gyro_Z_Ori - Gyro_Z_Offset);

    ORI_Gyrox = Gyro_X_Ori;
    ORI_Gyroy = Gyro_Y_Ori;
    ORI_Gyroz = Gyro_Z_Ori;

    ORI_Accx = Acc_X_Ori;
    ORI_Accy = Acc_Y_Ori;
    ORI_Accz = Acc_Z_Ori;
    /********************************************************/
    if (!butt_flag)
    {
        gyro_x.in = Gyro_X_Ori;
        LpFilter(&gyro_x);
        Gyro_X_Ori = gyro_x.out;

        gyro_y.in = Gyro_Y_Ori;
        LpFilter(&gyro_y);
        Gyro_Y_Ori = gyro_y.out;

        gyro_z.in = Gyro_Z_Ori;
        LpFilter(&gyro_z);
        Gyro_Z_Ori = gyro_z.out;

        // 低通滤波
        acc_x.in = Acc_X_Ori;
        LpFilter(&acc_x);
        Acc_X_Ori = acc_x.out;

        acc_y.in = Acc_Y_Ori;
        LpFilter(&acc_y);
        Acc_Y_Ori = acc_y.out;

        acc_z.in = Acc_Z_Ori;
        LpFilter(&acc_z);
        Acc_Z_Ori = acc_z.out;
    }
		        gyro_z.in = Gyro_Z_Ori;
        LpFilter(&gyro_z);
        Gyro_Z_Ori = gyro_z.out;
		        acc_z.in = Acc_Z_Ori;
        LpFilter(&acc_z);
        Acc_Z_Ori = acc_z.out;
    /*************************************************************/

    ////////////////加入butterworth滤波器/////////////////////////////

    /////////////////////////////////////////////////
    // Low pass filtering
    if (butt_flag)
    {
        Filters.GyroxLPF.input = Gyro_X_Ori;
        Filters.GyroyLPF.input = Gyro_Y_Ori;
 //       Filters.GyrozLPF.input = Gyro_Z_Ori;
        Butterworth50HzLPF(&Filters.GyroxLPF);
        Butterworth50HzLPF(&Filters.GyroyLPF);
 //       Butterworth50HzLPF(&Filters.GyrozLPF);

        Gyro_X_Ori = Filters.GyroxLPF.output;
        Gyro_Y_Ori = Filters.GyroyLPF.output;
//        Gyro_Z_Ori = Filters.GyrozLPF.output;

        Filters.AccxLPF.input = Acc_X_Ori;
        Filters.AccyLPF.input = Acc_Y_Ori;
 //       Filters.AcczLPF.input = Acc_Z_Ori;
        Butterworth30HzLPF(&Filters.AccxLPF);
        Butterworth30HzLPF(&Filters.AccyLPF);
  //      Butterworth30HzLPF(&Filters.AcczLPF);

        Acc_X_Ori = Filters.AccxLPF.output;
        Acc_Y_Ori = Filters.AccyLPF.output;
  //      Acc_Z_Ori = Filters.AcczLPF.output;
    }
    // 陀螺仪加死区	易出现NaN
    Gyro_X_Ori = Limit_Zero(Gyro_X_Ori, 0.002f);
    Gyro_Y_Ori = Limit_Zero(Gyro_Y_Ori, 0.002f);
    Gyro_Z_Ori = Limit_Zero(Gyro_Z_Ori, 0.002f);

    // 单位转换
    Gyro_X_Real = Gyro_X_Ori * 0.0174533f;
    Gyro_Y_Real = Gyro_Y_Ori * 0.0174533f;
    Gyro_Z_Real = Gyro_Z_Ori * 0.0174533f;

    Acc_X_Real = Acc_X_Ori;
    Acc_Y_Real = Acc_Y_Ori;
    Acc_Z_Real = Acc_Z_Ori;

    G_Cal = sqrt(Acc_X_Real * Acc_X_Real + Acc_Y_Real * Acc_Y_Real + Acc_Z_Real * Acc_Z_Real);

    if (G_Cal > 6000 * 1.74f)
        Warning = TRUE;
    else if (!(Gyro_X_Real < 2000 && Gyro_X_Real > -2000))
        Warning = TRUE;
    else
        Warning = FALSE;

    if (imu_mode == INIT)
        imu_mode = NORMAL;
    else if (imu_mode == CALIBRATION)
        imu_mode = CALIBRATION;
}

void LpFilter(ST_LPF *lpf) // 用于陀螺仪
{
    float fir_a = 1 / (1 + lpf->off_freq * lpf->samp_tim);
    lpf->preout = lpf->out;
    lpf->out = fir_a * lpf->preout + (1 - fir_a) * lpf->in;
}

/*Generated by:   http://www-users.cs.york.ac.uk/~fisher/mkfilter
filtertype = Butterworth    passtype = Lowpass   ripple =   order = 3
samplerate = 1000   corner1 = 50  corner2 = adzero = logmin =     */
#define GAINBtw50hz (3.450423889e+02f)
void Butterworth50HzLPF(Bw50HzLPFTypeDef *pLPF)
{
    pLPF->xv[0] = pLPF->xv[1];
    pLPF->xv[1] = pLPF->xv[2];
    pLPF->xv[2] = pLPF->xv[3];
    pLPF->xv[3] = pLPF->input / GAINBtw50hz;
    pLPF->yv[0] = pLPF->yv[1];
    pLPF->yv[1] = pLPF->yv[2];
    pLPF->yv[2] = pLPF->yv[3];
    pLPF->yv[3] = (pLPF->xv[0] + pLPF->xv[3]) + 3 * (pLPF->xv[1] + pLPF->xv[2]) + (0.5320753683f * pLPF->yv[0]) + (-1.9293556691f * pLPF->yv[1]) + (2.3740947437f * pLPF->yv[2]);

    pLPF->output = pLPF->yv[3];
}

/* Digital filter designed by mkfilter/mkshape/gencode   A.J. Fisher
   Command line: /www/usr/fisher/helpers/mkfilter -Bu -Lp -o 2 -a 3.0000000000e-02 0.0000000000e+00 -l */
// #define NZEROS 2   #define NPOLES 2  static float xv[NZEROS+1], yv[NPOLES+1];
#define GAINBtw30Hz 1.278738361e+02f
void Butterworth30HzLPF(Bw30HzLPFTypeDef *pLPF)
{
    pLPF->xv[0] = pLPF->xv[1];
    pLPF->xv[1] = pLPF->xv[2];
    pLPF->xv[2] = pLPF->input / GAINBtw30Hz;
    pLPF->yv[0] = pLPF->yv[1];
    pLPF->yv[1] = pLPF->yv[2];
    pLPF->yv[2] = (pLPF->xv[0] + pLPF->xv[2]) + 2 * pLPF->xv[1] + (-0.7660066009f * pLPF->yv[0]) + (1.7347257688f * pLPF->yv[1]);
    pLPF->output = pLPF->yv[2];
}
