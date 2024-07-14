#ifndef _MOTOR_
#define _MOTOR_
#pragma once
#include <Definitions.h>

#define eposmotor1_default {0,0,"EPOS4","MAXON SERIAL V2","USB","USB0",1,MT_DC_MOTOR,2170,10000,422,0,\
							10000,1000,1000,102400,500,0,HM_NEGATIVE_LIMIT_SWITCH,20000,0,0,\
							7000,17000,17000,0,\
							0,450,0,\
							0,0,0,0,0,0,0,0,0,0}
#define eposmotor2_default {0,0,"EPOS4","CANopen","","",2,MT_EC_BLOCK_COMMUTATED_MOTOR,2800,10000,175,7,\
							20000,2000,2000,43008,500,0,HM_NEGATIVE_LIMIT_SWITCH,20000,0,0,\
							14000,10000,10000,0,\
							0,90,0,\
							0,0,0,0,0,0,0,0,0,0}
#define tmotor1_default {0,0,0,0,0,0,0,0,0,0}
#define tmotor2_default {0,0,0,0,0,0,0,0,0,0}

typedef struct {
	// 기본설정
	HANDLE keyHandle;							// Device Handle
	HANDLE new_keyHandle;
	char deviceName[6];							// 장치 이름
	char ProtocolStackName[16];					// 프로토콜 이름
	char interfaceName[4];                      // 인터페이스 이름
	char portName[5];							// 포트 이름
	WORD NodeId;                                // 노드 번호
	WORD MotorType;								// epos모터 타입
	WORD NominalCurrent;                        // Nominal Current
	WORD MaxOutputCurrent;                      // Max Output Current
	WORD ThermalTimeConstant;                   // Thermal Time Constant Winding
	BYTE NbOfPolePairs;							// Number of Pole Pairs

	// Homing Mode
	DWORD HomingAcceleration;					// Homing 가속도
	DWORD SpeedSwitch;							// Speed for switch search
	DWORD SpeedIndex;							// Speed for zero search
	long HomeOffset;
	WORD CurrentThreshold;
	long HomePosition;
	char HomingMethod;
	DWORD Timeout;								// Homing 대기시간
	BOOL HomingAttained;						// 0 = Not Complete, 1= Completed
	BOOL HomingError;							// 0 = No Error, 1 = Error

	// Profile Position Mode
	DWORD ProfileVelocity;                      // epos모터 속도
	DWORD ProfileAcceleration;                  // epos모터 가속도
	DWORD ProfileDeceleration;                  // epos모터 감속도
	long TargetPosition;                        // epos모터 목표위치 초기화(inc)

	// 범위 제한
	double low_limit;							// epos모터 하한
	double high_limit;							// epos모터 상한
	double max_limit;							// epos모터 구동상한

	// Trajectory
	double target;								// epos모터 목표타깃
	double last_target;							// epos모터 최신타깃
	double velocity;							// Over Fold 구간 epos모터 속도
	double last_velocity;						// Over Fold 구간 epos모터 최신속도
	double velocity_temp;						// Over Fold 구간 epos모터 마지막 속도 
	double acceleration;						// Over Fold 구간 epos모터 가속도
	double deceleration;						// Over Fold 구간 epos모터 감속도
	double delta;								// Over Fold 구간 epos모터 변화량
	long actual_position;						// epos모터 실제위치
	double offset_position;						// epos모터 오프셋위치
}EPOS_Motor;
typedef struct {
	double velocity;							// t모터 속도
	double acceleration;						// t모터 가속도
	uint32_t velocity_temp;						// t모터 속도 / 10^3
	uint32_t acceleration_temp;					// t모터 가속도 / 10^3
	double angle;								// t모터 각도
	double position;							// t모터 Position
	uint32_t position_temp;						// 임시 계산용 t모터1 Position
	double actual_position;						// t모터 실제 Position
	uint32_t actual_position_temp;				// 임시 계산용 t모터1 실제 Position
	double last_position;						// t모터 최신 Position
}T_Motor;
#endif