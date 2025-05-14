/*
 * 	app.c
 *	����������ϵͳӦ�ù��ܴ���ָ�������ID����������ƣ�Э�齻��,���������
 *	����ʱ��: 2025.05.08
 *	������Ա��WUB
 *	������˾: ��ɽ��ά
 */
#include "app.h"
Target_S targetData;    // Ŀ�����ݻ�����
Target_S targetDataBAK; // Ŀ�����ݱ��ݻ�����

void dataRead(void);  // ���ݶ�ȡ����
void dataWrite(void); // ����д�뺯��
void IDClear(void);   // ID�������
void zeroSet(void);   // ������ú���
void IDSet(u8 ID);
/***********************************************
��������: void Init_sysApp(void)
�������ܣ���ʼ�������룬У��ȷ�ϣ�ϵͳ����׼����
��ڲ�������
���ڲ�������
��ע��
************************************************/
void Init_sysApp(void) // ϵͳ��ʼ������
{
    // targetData.ID = 0;            // ����ID Ĭ��ID��Ч��
    // targetData.new = 0;           // ����δִ�б�־λ���
    // targetData.moto_zeroData = 0; // ���������Ĭ��Ϊ��
    // dataRead();                   // ���ݶ�ȡ����
}

/***********************************************
�������ƣ�void SysProcess(void)
�������ܣ�ϵͳָ�����
��ڲ�������
���ڲ�������
��ע��5ms ���ڵ���
************************************************/
void SysProcess(void) // ϵͳָ�����
{
    // if (targetData.new) // ����������ݵ���
    // {
    //     targetData.new = 0;        // ��������ݱ�־λ
    //     if (targetData.ID == 0x00) // ID��Ч
    //     {
    //         if (targetData.CT != CT_MODE_IDSET) // ����Ĭ��ID
    //             return;                         // ��IDʱ����ִ�������������ָ�
    //     }
    //     switch (targetData.CT)
    //     {
    //     case CT_MODE_RUN:                           // ����ģʽ
    //         moto_ABSPRun(targetData.data, GORUNSP); // �������λ�����з�ʽ
    //         break;

    //     case CT_MODE_PSET:               // λ������ģʽ
    //         if (targetData.data == 0x11) // ��λ����ָ��
    //         {
    //             zeroSet(); // ������ú���
    //         }
    //         else if (targetData.data == 0x22) // �豸����
    //         {
    //             moto_ABSPRun(0, GETZEROSP); // ����λ
    //         }
    //         break;

    //     case CT_MODE_IDSET:           // ID����ģʽ
    //         if (targetData.ID == 0x0) // ID����ָ��
    //         {
    //             IDSet(targetData.data); // ID���ú���
    //         }
    //         else if (targetData.data == 0x11) // ID��λָ��
    //         {
    //             IDClear(); // ID�������
    //         }
    //         break;

    //     case CT_MODE_STA:                // ״̬��ѯģʽ
    //         if (targetData.data == 0x11) // ��ǰλ�÷���
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
�������ƣ�void IDClear(void)
�������ܣ�ID�������
��ڲ�������
���ڲ�������
��ע��
************************************************/
void IDClear(void) // ����������
{
    // targetData.ID = 0; // ID���
    // dataWrite();       // ����д�뺯��
	// IDOUT = 0;
}

/***********************************************
�������ƣ�void IDSet(u8 ID)
�������ܣ�ID���ú���
��ڲ�������
���ڲ�������
��ע��
************************************************/
void IDSet(u8 ID) // ID���ú���
{
    // if (IDIN) // Ĭ�ϸ�����Ϊ��Ч�ߵ�ƽ
    // {
    //     targetData.ID = ID; // ID����
    //     dataWrite();        // ����д�뺯��
    //     Send_TX_int();      // ���ͷ�������
	// 	IDOUT = 1;
   // }
}

/***********************************************
�������ƣ�void zeroSet(void)
�������ܣ���ǰλ������Ϊ���
��ڲ�������
���ڲ�������
��ע��
************************************************/
void zeroSet(void) // ������ú���
{
    // Read_Position();                          // ��ȡ����������
    // moto_zeroData = encData[0];               // ���������ƫ������
    // targetData.moto_zeroData = moto_zeroData; // �������д��
    // dataWrite();                              // ����д�뺯��
    // moto_ABSPRun(ZEROTESTANG, ZEROTESTSP);    // ���ƫת90�ȡ�
}

/***********************************************
�������ƣ�void dataRead(void)
�������ܣ�ϵͳ������ȡ����
��ڲ�������
���ڲ�����ID����λ���ݶ�ȡ����ϵͳ������
��ע��
************************************************/
void dataRead(void) // ���ݶ�ȡ����
{
    // for (char i = 0; i < sizeof(Target_S) / 2; i++)
    // {
    //     *((u16 *)&targetDataBAK + i) = Memory_Read(targetBakAddress + i); // ��ȡ����
    // }

    // u8 crc = 0; // CRCУ��ֵ
    // u8 *TED = (u8 *)&targetDataBAK;
    // for (char i = 0; i < sizeof(Target_S) - 1; i++)
    // {
    //     crc += TED[i]; // CRCУ��
    // }
    // if (crc == targetDataBAK.sumCheck)
    // {
    //     targetData.moto_zeroData = targetDataBAK.moto_zeroData; // ��ȡ��λ��Ϣ
    //     targetData.ID = targetDataBAK.ID;                       // ID��ȡ
    //     moto_zeroData = targetDataBAK.moto_zeroData;            // ���������ƫ������
    // }
}

/***********************************************
�������ƣ�void dataWrite(void)
�������ܣ�ϵͳ����д�뺯��
��ڲ�������
���ڲ�����ID����λ����д��ϵͳeepron
��ע��
************************************************/
void dataWrite(void) // ����д�뺯��
{
    // u8 crc = 0; // CRCУ��ֵ
    // u8 *TED = (u8 *)&targetDataBAK;
    // targetDataBAK.ID = targetData.ID;                       // IDд��
    // targetDataBAK.moto_zeroData = targetData.moto_zeroData; // ���������ƫ������

    // for (char i = 0; i < sizeof(Target_S) - 1; i++)
    // {
    //     crc += TED[i]; // CRCУ��
    // }
    // targetDataBAK.sumCheck = crc; // ��У��
    // for (char i = 0; i < sizeof(Target_S) / 2; i++)
    // {
    //     Memory_Write(targetBakAddress + i, *((u16 *)&targetDataBAK + i)); // д������
    // }
  ;
}
