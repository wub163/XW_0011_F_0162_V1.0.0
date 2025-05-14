/*
 * 	app_config.h
 *	功能描述：工程关键性能类调整配置文件
 *	创建时间: 2025.04.27
 *	开发人员：WUB
 *	开发公司: 昆山祥维
 */

#ifndef _APP_CONFIG_H
#define _APP_CONFIG_H

#define targetBakAddress 0x00 // 系统数据保存首地址
#define M_encode 4096         // 编码器设定精度4096
#define M_CPulse 3200         // 电机单圈脉冲数(控制)

#define MSInc 1      // 电机提速系数 20ms一次（1-20）
#define MSDec 3      // 电机减速系数 20ms一次（1-20）
#define DECSPPOS 100 // 电机减速位置360/3200

#define GETZEROSP 20  // 回零位速度2rps
#define GORUNSP 40    // 运行速度4rps
#define ZEROTESTSP 30 // 零点试运行速度3rps

#define ZEROTESTANG 24 // 零点试运行角度120度
#define STOPFREE 1     // 停车时电机释放
#define STOPTIME 30    // 停车时电机释放延时(1-250)*11ms
#define VMAX 60        // 电机最大速度
#define VMIN 3         // 电机最小速度

#endif
