/*
 * 	app_config.h
 *	�������������̹ؼ���������������ļ�
 *	����ʱ��: 2025.04.27
 *	������Ա��WUB
 *	������˾: ��ɽ��ά
 */

#ifndef _APP_CONFIG_H
#define _APP_CONFIG_H

#define targetBakAddress 0x00 // ϵͳ���ݱ����׵�ַ
#define M_encode 4096         // �������趨����4096
#define M_CPulse 3200         // �����Ȧ������(����)

#define MSInc 1      // �������ϵ�� 20msһ�Σ�1-20��
#define MSDec 3      // �������ϵ�� 20msһ�Σ�1-20��
#define DECSPPOS 100 // �������λ��360/3200

#define GETZEROSP 20  // ����λ�ٶ�2rps
#define GORUNSP 40    // �����ٶ�4rps
#define ZEROTESTSP 30 // ����������ٶ�3rps

#define ZEROTESTANG 24 // ��������нǶ�120��
#define STOPFREE 1     // ͣ��ʱ����ͷ�
#define STOPTIME 30    // ͣ��ʱ����ͷ���ʱ(1-250)*11ms
#define VMAX 60        // �������ٶ�
#define VMIN 3         // �����С�ٶ�

#endif
