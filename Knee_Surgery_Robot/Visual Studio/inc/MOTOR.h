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
	// �⺻����
	HANDLE keyHandle;							// Device Handle
	HANDLE new_keyHandle;
	char deviceName[6];							// ��ġ �̸�
	char ProtocolStackName[16];					// �������� �̸�
	char interfaceName[4];                      // �������̽� �̸�
	char portName[5];							// ��Ʈ �̸�
	WORD NodeId;                                // ��� ��ȣ
	WORD MotorType;								// epos���� Ÿ��
	WORD NominalCurrent;                        // Nominal Current
	WORD MaxOutputCurrent;                      // Max Output Current
	WORD ThermalTimeConstant;                   // Thermal Time Constant Winding
	BYTE NbOfPolePairs;							// Number of Pole Pairs

	// Homing Mode
	DWORD HomingAcceleration;					// Homing ���ӵ�
	DWORD SpeedSwitch;							// Speed for switch search
	DWORD SpeedIndex;							// Speed for zero search
	long HomeOffset;
	WORD CurrentThreshold;
	long HomePosition;
	char HomingMethod;
	DWORD Timeout;								// Homing ���ð�
	BOOL HomingAttained;						// 0 = Not Complete, 1= Completed
	BOOL HomingError;							// 0 = No Error, 1 = Error

	// Profile Position Mode
	DWORD ProfileVelocity;                      // epos���� �ӵ�
	DWORD ProfileAcceleration;                  // epos���� ���ӵ�
	DWORD ProfileDeceleration;                  // epos���� ���ӵ�
	long TargetPosition;                        // epos���� ��ǥ��ġ �ʱ�ȭ(inc)

	// ���� ����
	double low_limit;							// epos���� ����
	double high_limit;							// epos���� ����
	double max_limit;							// epos���� ��������

	// Trajectory
	double target;								// epos���� ��ǥŸ��
	double last_target;							// epos���� �ֽ�Ÿ��
	double velocity;							// Over Fold ���� epos���� �ӵ�
	double last_velocity;						// Over Fold ���� epos���� �ֽżӵ�
	double velocity_temp;						// Over Fold ���� epos���� ������ �ӵ� 
	double acceleration;						// Over Fold ���� epos���� ���ӵ�
	double deceleration;						// Over Fold ���� epos���� ���ӵ�
	double delta;								// Over Fold ���� epos���� ��ȭ��
	long actual_position;						// epos���� ������ġ
	double offset_position;						// epos���� ��������ġ
}EPOS_Motor;
typedef struct {
	double velocity;							// t���� �ӵ�
	double acceleration;						// t���� ���ӵ�
	uint32_t velocity_temp;						// t���� �ӵ� / 10^3
	uint32_t acceleration_temp;					// t���� ���ӵ� / 10^3
	double angle;								// t���� ����
	double position;							// t���� Position
	uint32_t position_temp;						// �ӽ� ���� t����1 Position
	double actual_position;						// t���� ���� Position
	uint32_t actual_position_temp;				// �ӽ� ���� t����1 ���� Position
	double last_position;						// t���� �ֽ� Position
}T_Motor;
#endif