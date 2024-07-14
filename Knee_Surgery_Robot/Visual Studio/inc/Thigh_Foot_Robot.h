#ifndef _ROBOT_
#define _ROBOT_
#pragma once
#define _WINSOCKAPI_
#include <windows.h>
#include <WinSock2.h>
#include <PCANBasic.h>
#include <thread>
#include <iostream>
#include <MOTOR.h>
#include <global_variables.h>

#define BIRRATE PCAN_BAUD_1M

class Thigh_Foot_Robot {
private:
	const TPCANHandle PcanHandle = PCAN_USBBUS1;
	TPCANMsg CAN_Transmit_buf[1];						// CAN��� �۽Ź���
	TPCANMsg CAN_Receive_buf[1];						// CAN��� ���Ź���
	TPCANTimestamp CANTimeStamp;
	std::thread* _t;

	void CAN_Data_reset();								// CAN��� ���� �ʱ�ȭ
	void CAN_Thread();									// �������� �Է´�� ������
	void CAN_Open_Thread();								// ������ ����
	void CAN_Close_Thread();							// ������ ����
	void CAN_Data_Read();								// CAN����� ���� ������ ����
	void CAN_Send();									// CAN����� ���� ������ �۽�

	void foot_setup();									// �߹�ħ�� �⺻���� �� �ʱ�ȭ
	void Homing_Mode();									// epos����1,2 Homing Mode
	void Profile_Position_Mode();						// epos����1,2 Profile Position Mode
	void receive_body_info();							// ��ü���� ����

	// reset_and_offset()
	bool validate_thigh_range(double x, double y);		// ����� ��ǥ�� ���� ���� �ִ��� Ȯ��
	bool validate_foot_range(double e1, double e2);		// �߹�ħ�� ��ǥ�� ���� ���� �ִ��� Ȯ��
	void move_eposmotor_PPM();							// epos����1,2 PPM ����

	// while_loop()
	void knee_actual_value();							// ���� ������
	void read_motor_actual_position();                  // t/epos����1,2 ���� Position ����
	void forward_kinematics();                          // t����1,2 ���� Position -> ���� ������

	bool CAN_threadFlag;								// �������� �Է´�� ������ Flag
	double angle_input;									// �������� �Է�
	bool new_angle_input;								// �������� �Է� Flag
	bool knee_up;										// ���� �ø� Flag
	bool knee_down;										// ���� ���� Flag
	bool check_current_value;							// ���簪 Ȯ�� Flag
	bool check_last_target;								// �ֽ�Ÿ�� Ȯ�� Flag
	bool button_pressed;								// ��ư ���� Flag

	int flag_offset = 0;								// ������ Flag
	bool flag_offset_complete = false;					// ������ �Ϸ� Flag

public:
	Thigh_Foot_Robot();									// �����/�� ��ħ�� ������

	// main��
	void reset_and_offset();							// �ʱ�ȭ �� ������ ����
	void while_loop();									// While�� ����
	int flag_while = 0;									// While�� Flag

	// 10ms Ÿ�̸�
	void move_Offset();									// t����1,2 Offset ����
	void generate_sector_trajectory();					// Sector Trajectory ����
	void move_Sector_Trajectory();						// Sector Trajectory ����
	void move_Sector_Halt_Trajectory();					// Sector Halt Trajectory ����
	void generate_over_fold_trajectory();				// Over Fold Trajectory ����
	void move_Over_Fold_Trajectory();					// Over Fold Trajectory ����
	void move_Over_Fold_Halt_Trajectory();				// Over Fold Halt Trajectory ����

	// 50ms Ÿ�̸�
	void move_Both_Trajectory();						// epos����1,2 Trajectory ����
};
#endif