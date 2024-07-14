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
	void CAN_Thread();			// 무릎각도 입력대기 쓰레드2
	void open_Thread();			// 쓰레드 실행
	void close_Thread();		// 쓰레드 종료
	bool threadFlag;			// 쓰레드 Flag
	double angle_input;			// 무릎각도 입력
	bool new_angle_input;		// 무릎각도 입력 Flag
};
