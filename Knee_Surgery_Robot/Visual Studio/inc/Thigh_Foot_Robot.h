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
	TPCANMsg CAN_Transmit_buf[1];						// CAN통신 송신버퍼
	TPCANMsg CAN_Receive_buf[1];						// CAN통신 수신버퍼
	TPCANTimestamp CANTimeStamp;
	std::thread* _t;

	void CAN_Data_reset();								// CAN통신 버퍼 초기화
	void CAN_Thread();									// 무릎각도 입력대기 쓰레드
	void CAN_Open_Thread();								// 쓰레드 실행
	void CAN_Close_Thread();							// 쓰레드 종료
	void CAN_Data_Read();								// CAN통신을 통해 데이터 수신
	void CAN_Send();									// CAN통신을 통해 데이터 송신

	void foot_setup();									// 발받침대 기본설정 및 초기화
	void Homing_Mode();									// epos모터1,2 Homing Mode
	void Profile_Position_Mode();						// epos모터1,2 Profile Position Mode
	void receive_body_info();							// 신체정보 수신

	// reset_and_offset()
	bool validate_thigh_range(double x, double y);		// 허벅지 좌표가 범위 내에 있는지 확인
	bool validate_foot_range(double e1, double e2);		// 발받침대 좌표가 범위 내에 있는지 확인
	void move_eposmotor_PPM();							// epos모터1,2 PPM 구동

	// while_loop()
	void knee_actual_value();							// 무릎 실제값
	void read_motor_actual_position();                  // t/epos모터1,2 실제 Position 수신
	void forward_kinematics();                          // t모터1,2 실제 Position -> 무릎 실제값

	bool CAN_threadFlag;								// 무릎각도 입력대기 쓰레드 Flag
	double angle_input;									// 무릎각도 입력
	bool new_angle_input;								// 무릎각도 입력 Flag
	bool knee_up;										// 무릎 올림 Flag
	bool knee_down;										// 무릎 내림 Flag
	bool check_current_value;							// 현재값 확인 Flag
	bool check_last_target;								// 최신타깃 확인 Flag
	bool button_pressed;								// 버튼 눌림 Flag

	int flag_offset = 0;								// 오프셋 Flag
	bool flag_offset_complete = false;					// 오프셋 완료 Flag

public:
	Thigh_Foot_Robot();									// 허벅지/발 받침대 생성자

	// main문
	void reset_and_offset();							// 초기화 및 오프셋 설정
	void while_loop();									// While문 실행
	int flag_while = 0;									// While문 Flag

	// 10ms 타이머
	void move_Offset();									// t모터1,2 Offset 실행
	void generate_sector_trajectory();					// Sector Trajectory 생성
	void move_Sector_Trajectory();						// Sector Trajectory 실행
	void move_Sector_Halt_Trajectory();					// Sector Halt Trajectory 실행
	void generate_over_fold_trajectory();				// Over Fold Trajectory 생성
	void move_Over_Fold_Trajectory();					// Over Fold Trajectory 실행
	void move_Over_Fold_Halt_Trajectory();				// Over Fold Halt Trajectory 실행

	// 50ms 타이머
	void move_Both_Trajectory();						// epos모터1,2 Trajectory 실행
};
#endif