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

// ��ſ� ���Ǵ� ���� ���۵��� �ʱ�ȭ�ϱ� ���� �Լ�
void CAN::Data_reset() {
	CAN_Reset(PcanHandle);
}

// CAN����� ���� ������ ����
void CAN::CAN_Data_Read() {
	CAN_Read(PcanHandle, Receive_buf, &CANTimeStamp);
}

// CAN����� ���� ������ �۽�
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


// �������� �Է´�� ������
void CAN::CAN_Thread() {
	while (threadFlag) {
		cin >> angle_input;
		new_angle_input = true;
	}
}
// ������ ����
void CAN::open_Thread() {
	threadFlag = true;
	_t = new std::thread(&CAN::CAN_Thread, this);
}
// ������ ����
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
				// �����͸� Parsing(�з�)�ϴ� �Լ��� ȣ���ϸ�, �ùٸ� �з��� �� ��쿡�� �ݺ� ����
				if (CAN_Parsing(stsResult, Receive_buf[i]))
					break;
			}
		}
	}
	// �Էµ� ���۸� ����ϸ鼭 Ȯ���ϱ� ���� �κ�
	// printf("Data : %x %x %x %x %x %x %x %x                \n", Receive_buf->DATA[0], Receive_buf->DATA[1], Receive_buf->DATA[2], Receive_buf->DATA[3], Receive_buf->DATA[4], Receive_buf->DATA[5], Receive_buf->DATA[6], Receive_buf->DATA[7]);
}

// �Ҹ���, ������ ����
CAN::~CAN() {
	threadFlag1 = false;
	_t1->join();
	delete _t1;
}

// CAN����� ���� ���� �����͸� �з��ϱ� ���� �Լ�
bool CAN::CAN_Parsing(bool flag, TPCANMsg msg) {
	return true;
}
*/