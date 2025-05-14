#ifndef _APP_H
#define _APP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "application.h"
// #define targetBakAddress 0x00 // Ŀ�����ݵ�ַ�����ַ��

    enum CT_mode
    {
        CT_MODE_RUN = 0x00,   // ����ģʽ
        CT_MODE_PSET = 0x01,  // λ������ģʽ
        CT_MODE_IDSET = 0x02, // ID����ģʽ
        CT_MODE_STA = 0x03,   // ״̬��ѯģʽ
    };

    typedef struct
    {
        uint8_t ID;             // ����ID
        uint8_t CT;             // ������
        uint8_t data;           // ��������
        uint8_t new;            // ����δִ��
        uint16_t Pdata;         // ��ǰλ��(��2048)
        uint16_t moto_zeroData; // ���������ƫ������
        uint8_t reserve1;       // ����
        uint8_t sumCheck;       // ��У��
    } Target_S;

    extern Target_S targetData; // Ŀ�����ݻ�����
    void Init_sysApp(void);     // ϵͳ��ʼ������
    void SysProcess(void);      // ϵͳָ�����

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif // _APP_H