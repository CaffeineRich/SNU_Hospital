#pragma once
#include <PCANBasic.h>
#include <thread>
#include <iostream>


#define BIRRATE PCAN_BAUD_1M
#define CAN_BUF_SIZE		1

class CAN
{
private:
	const TPCANHandle PcanHandle = PCAN_USBBUS1;
	const bool IsFD = false;
	TPCANTimestamp CANTimeStamp;
	void Data_reset();
	std::thread* _t;
	//bool CAN_Parsing(bool flag, TPCANMsg msg);
	//~CAN();
public:
	char buf[1024];
	TPCANMsg Transmit_buf[CAN_BUF_SIZE];
	TPCANMsg Receive_buf[CAN_BUF_SIZE];
	CAN();
	void CAN_Data_Read();
	void CAN_Send();
	void CAN_Thread();			// �������� �Է´�� ������2
	void open_Thread();			// ������ ����
	void close_Thread();		// ������ ����
	bool threadFlag;			// ������ Flag
	double angle_input;			// �������� �Է�
	bool new_angle_input;		// �������� �Է� Flag
};
