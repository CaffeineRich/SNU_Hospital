#include <CAN.h>
using namespace std;

CAN::CAN() {
	TPCANStatus res;
	res = CAN_Initialize(PcanHandle, BIRRATE);
	if (res != PCAN_ERROR_OK) {
		cout << "Can not initialize. Please check the defines in the code.\n";
	}
	else {
		cout << "Successfully initialized.\n";
		Transmit_buf->ID = 0x321;
		Transmit_buf->LEN = 8;
		Receive_buf->LEN = 8;
		Data_reset();
		open_Thread();
	}
}

// 통신에 사용되는 각종 버퍼들을 초기화하기 위한 함수
void CAN::Data_reset() {
	CAN_Reset(PcanHandle);
}

// CAN통신을 통해 데이터 수신
void CAN::CAN_Data_Read() {
	CAN_Read(PcanHandle, Receive_buf, &CANTimeStamp);
}

// CAN통신을 통해 데이터 송신
void CAN::CAN_Send() {
	TPCANStatus res;
	for (int i = 0 ; i < CAN_BUF_SIZE ; i++) {
		res = CAN_Write(PcanHandle, &Transmit_buf[i]);
		if (res != PCAN_ERROR_OK) {
			std::cout << "Data Send Fail.\n";
			system("PAUSE");
			return;
		}
	}
}


// 무릎각도 입력대기 쓰레드
void CAN::CAN_Thread() {
	while (threadFlag) {
		cin >> angle_input;
		new_angle_input = true;
	}
}
// 쓰레드 실행
void CAN::open_Thread() {
	threadFlag = true;
	_t = new std::thread(&CAN::CAN_Thread, this);
}
// 쓰레드 종료
void CAN::close_Thread() {
	threadFlag = false;
	_t->detach();
}

/*
void CAN::CAN_Data_Read() {
	Data_reset();
	for (int i = 0; i < CAN_BUF_SIZE; i++) {
		while (1) {
			static TPCANStatus stsResult = CAN_Read(PcanHandle, &Receive_buf[i], &CANTimeStamp);
			if (stsResult != PCAN_ERROR_QRCVEMPTY) {
				// 데이터를 Parsing(분류)하는 함수를 호출하며, 올바른 분류가 된 경우에만 반복 종료
				if (CAN_Parsing(stsResult, Receive_buf[i]))
					break;
			}
		}
	}
	// 입력된 버퍼를 출력하면서 확인하기 위한 부분
	// printf("Data : %x %x %x %x %x %x %x %x                \n", Receive_buf->DATA[0], Receive_buf->DATA[1], Receive_buf->DATA[2], Receive_buf->DATA[3], Receive_buf->DATA[4], Receive_buf->DATA[5], Receive_buf->DATA[6], Receive_buf->DATA[7]);
}

// 소멸자, 쓰레드 종료
CAN::~CAN() {
	threadFlag1 = false;
	_t1->join();
	delete _t1;
}

// CAN통신을 통해 받은 데이터를 분류하기 위한 함수
bool CAN::CAN_Parsing(bool flag, TPCANMsg msg) {
	return true;
}
*/