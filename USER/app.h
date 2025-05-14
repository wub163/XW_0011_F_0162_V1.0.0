#ifndef _APP_H
#define _APP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "application.h"
// #define targetBakAddress 0x00 // 目标数据地址保存地址区

    enum CT_mode
    {
        CT_MODE_RUN = 0x00,   // 运行模式
        CT_MODE_PSET = 0x01,  // 位置配置模式
        CT_MODE_IDSET = 0x02, // ID配置模式
        CT_MODE_STA = 0x03,   // 状态查询模式
    };

    typedef struct
    {
        uint8_t ID;             // 本机ID
        uint8_t CT;             // 控制字
        uint8_t data;           // 控制数据
        uint8_t new;            // 数据未执行
        uint16_t Pdata;         // 当前位置(±2048)
        uint16_t moto_zeroData; // 编码器零点偏移数据
        uint8_t reserve1;       // 备用
        uint8_t sumCheck;       // 和校验
    } Target_S;

    extern Target_S targetData; // 目标数据缓存区
    void Init_sysApp(void);     // 系统初始化函数
    void SysProcess(void);      // 系统指令处理函数

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif // _APP_H