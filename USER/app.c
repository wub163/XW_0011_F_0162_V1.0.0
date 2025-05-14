/*
 * 	app.c
 *	功能描述：系统应用功能处理，指令解析，ID处理，电机控制，协议交互,参数保存等
 *	创建时间: 2025.05.08
 *	开发人员：WUB
 *	开发公司: 昆山祥维
 */
#include "app.h"
Target_S targetData;    // 目标数据缓存区
Target_S targetDataBAK; // 目标数据备份缓存区

void dataRead(void);  // 数据读取函数
void dataWrite(void); // 数据写入函数
void IDClear(void);   // ID清除函数
void zeroSet(void);   // 零点设置函数
void IDSet(u8 ID);
/***********************************************
函数名称: void Init_sysApp(void)
函数功能：初始参数读入，校验确认，系统参数准备。
入口参数：无
出口参数：无
备注：
************************************************/
void Init_sysApp(void) // 系统初始化函数
{
    // targetData.ID = 0;            // 本机ID 默认ID无效。
    // targetData.new = 0;           // 数据未执行标志位清除
    // targetData.moto_zeroData = 0; // 编码器零点默认为零
    // dataRead();                   // 数据读取函数
}

/***********************************************
函数名称：void SysProcess(void)
函数功能：系统指令处理函数
入口参数：无
出口参数：无
备注：5ms 周期调用
************************************************/
void SysProcess(void) // 系统指令处理函数
{
    // if (targetData.new) // 如果有新数据到达
    // {
    //     targetData.new = 0;        // 清除新数据标志位
    //     if (targetData.ID == 0x00) // ID无效
    //     {
    //         if (targetData.CT != CT_MODE_IDSET) // 设置默认ID
    //             return;                         // 无ID时，不执行配置外的其它指令。
    //     }
    //     switch (targetData.CT)
    //     {
    //     case CT_MODE_RUN:                           // 运行模式
    //         moto_ABSPRun(targetData.data, GORUNSP); // 电机绝对位置运行方式
    //         break;

    //     case CT_MODE_PSET:               // 位置配置模式
    //         if (targetData.data == 0x11) // 零位配置指令
    //         {
    //             zeroSet(); // 零点设置函数
    //         }
    //         else if (targetData.data == 0x22) // 设备回零
    //         {
    //             moto_ABSPRun(0, GETZEROSP); // 回零位
    //         }
    //         break;

    //     case CT_MODE_IDSET:           // ID配置模式
    //         if (targetData.ID == 0x0) // ID配置指令
    //         {
    //             IDSet(targetData.data); // ID设置函数
    //         }
    //         else if (targetData.data == 0x11) // ID复位指令
    //         {
    //             IDClear(); // ID清除函数
    //         }
    //         break;

    //     case CT_MODE_STA:                // 状态查询模式
    //         if (targetData.data == 0x11) // 当前位置反馈
    //         {
    //             Send_TX_int();
    //         }
    //         break;

    //     default:
    //         break;
    //     }
    // }
}

/***********************************************
函数名称：void IDClear(void)
函数功能：ID清除函数
入口参数：无
出口参数：无
备注：
************************************************/
void IDClear(void) // 零点清除函数
{
    // targetData.ID = 0; // ID清除
    // dataWrite();       // 数据写入函数
	// IDOUT = 0;
}

/***********************************************
函数名称：void IDSet(u8 ID)
函数功能：ID设置函数
入口参数：无
出口参数：无
备注：
************************************************/
void IDSet(u8 ID) // ID设置函数
{
    // if (IDIN) // 默认浮空则为有效高电平
    // {
    //     targetData.ID = ID; // ID设置
    //     dataWrite();        // 数据写入函数
    //     Send_TX_int();      // 发送反馈数据
	// 	IDOUT = 1;
   // }
}

/***********************************************
函数名称：void zeroSet(void)
函数功能：当前位置设置为零点
入口参数：无
出口参数：无
备注：
************************************************/
void zeroSet(void) // 零点设置函数
{
    // Read_Position();                          // 读取编码器数据
    // moto_zeroData = encData[0];               // 编码器零点偏移数据
    // targetData.moto_zeroData = moto_zeroData; // 零点数据写入
    // dataWrite();                              // 数据写入函数
    // moto_ABSPRun(ZEROTESTANG, ZEROTESTSP);    // 电机偏转90度。
}

/***********************************************
函数名称：void dataRead(void)
函数功能：系统参数读取函数
入口参数：无
出口参数：ID，零位数据读取填入系统参数表
备注：
************************************************/
void dataRead(void) // 数据读取函数
{
    // for (char i = 0; i < sizeof(Target_S) / 2; i++)
    // {
    //     *((u16 *)&targetDataBAK + i) = Memory_Read(targetBakAddress + i); // 读取数据
    // }

    // u8 crc = 0; // CRC校验值
    // u8 *TED = (u8 *)&targetDataBAK;
    // for (char i = 0; i < sizeof(Target_S) - 1; i++)
    // {
    //     crc += TED[i]; // CRC校验
    // }
    // if (crc == targetDataBAK.sumCheck)
    // {
    //     targetData.moto_zeroData = targetDataBAK.moto_zeroData; // 获取零位信息
    //     targetData.ID = targetDataBAK.ID;                       // ID读取
    //     moto_zeroData = targetDataBAK.moto_zeroData;            // 编码器零点偏移数据
    // }
}

/***********************************************
函数名称：void dataWrite(void)
函数功能：系统参数写入函数
入口参数：无
出口参数：ID，零位数据写入系统eepron
备注：
************************************************/
void dataWrite(void) // 数据写入函数
{
    // u8 crc = 0; // CRC校验值
    // u8 *TED = (u8 *)&targetDataBAK;
    // targetDataBAK.ID = targetData.ID;                       // ID写入
    // targetDataBAK.moto_zeroData = targetData.moto_zeroData; // 编码器零点偏移数据

    // for (char i = 0; i < sizeof(Target_S) - 1; i++)
    // {
    //     crc += TED[i]; // CRC校验
    // }
    // targetDataBAK.sumCheck = crc; // 和校验
    // for (char i = 0; i < sizeof(Target_S) / 2; i++)
    // {
    //     Memory_Write(targetBakAddress + i, *((u16 *)&targetDataBAK + i)); // 写入数据
    // }
  ;
}
