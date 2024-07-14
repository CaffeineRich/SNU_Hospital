#ifndef _GLOBAL_
#define _GLOBAL_
#pragma once
#define PI 3.1415926						// ������

// Test ��ó����
#define Test 1
// 1: t/epos���� ����
// 2: t���� ����
// 3: epos���� ����
// 4: ����X

// Trajectory
class trajectory {
public:
	int cnt_NP = 1;							// Way Point ������ġ
	int number_of_path = 0;                 // Way Point ����
	bool over_fold;							// Over Fold ���� Flag
	int flag_move_sector_trajectory = 0;	// Sector Trajectory ���� Flag
	int flag_move_over_fold_trajectory = 0;	// Over Fold Trajectory ���� Flag
	int flag_sector_halt = 0;				// Sector Halt Trajectory ���� Flag
	int flag_over_halt = 0;					// Over Fold Halt Trajectory ���� Flag
	double division_sec = 0;				// Sector Trajectory ���ӱ���
	double division_over = 0;				// Over Fold Trajectory ���ӱ���
	double halt_delay = 0;					// Halt Trajectory ���ӱ���
	double tmotor_angle_addition = 0;		// t����1,2 Position ��
};

// ��� ��ü
class human_gogwanjeol {
public:
	double x = 0;							// ����� x��ǥ
	double y = 0;							// ����� y��ǥ
};
class human_leg {
public:
	double top = 0;							// ���ٸ� ����
	double bottom = 0;						// �Ʒ��ٸ� ����
	double GARO = 0;						// �ٸ� ���α���
	double SERO = 0;						// �ٸ� ���α���
	double angle = 0;						// ����~�߲�ġ�� ����� ���� ����
};
class human_knee {
public:
	double angle_target = 0;				// ���� ��ǥ����
	double angle_last_target = 0;			// ���� �ֽŰ���
	double angle_max = 0;					// ���� �ִ밢��
	double angle_min = 0;					// ���� �ּҰ���
	double angle_actual = 0;				// ���� ��������
	double x_target = 0;					// ���� x��ǥ ��ǥŸ��
	double x_last_target = 0;				// ���� x��ǥ �ֽ�Ÿ��
	double y_target = 0;					// ���� y��ǥ ��ǥŸ��
	double y_last_target = 0;				// ���� y��ǥ �ֽ�Ÿ��
	double x_actual = 0;					// ���� ���� x��ǥ
	double y_actual = 0;					// ���� ���� y��ǥ
};

// ����� ��ħ��
class thigh {
public:
	double x_target = 0;					// ����� x��ǥ ��ǥŸ��
	double x_last_target = 0;				// ����� x��ǥ �ֽ�Ÿ��
	double y_target = 0;					// ����� y��ǥ ��ǥŸ��
	double y_last_target = 0;				// ����� y��ǥ �ֽ�Ÿ��
	double radius = 0;						// ����� ������(�����~����� ����)
	double angle_actual = 0;				// �����~������� ����� ���� ���� ����
	double angle_target = 0;				// ����� ���� ��ǥŸ��
	double angle_last_target = 0;			// ����� ���� �ֽ�Ÿ��
	double angle_max = 0;					// ����� �ִ밢��
	double angle_min = 0;					// ����� �ּҰ���
	double angle_difference = 0;			// ����� ���� �� ��ȭ��
	double velocity = 0;					// ����� �ӵ�
	double last_velocity = 0;				// ����� �ֽżӵ�
	double velocity_temp = 0;				// ����� ������ �ӵ�
	double acceleration = 0;				// ����� ���ӵ�
	double deceleration = 0;				// ����� ���ӵ�
	double gap_btw_device = 0;				// ������� �ⱸ���� �Ÿ�
	double angle_btw_device = 0;			// ������� �ⱸ���� ����
};
class device {
public:
	const double x_def = 0;																						// �ⱸ x��ǥ �ʱⰪ
	const double y_def = 124;																					// �ⱸ y��ǥ �ʱⰪ
	const double leg_1 = 160;																					// a1 = 160mm
	const double leg_2 = 160;																					// a2 = 160mm
	double D_def = (x_def * x_def + y_def * y_def - leg_1 * leg_1 - leg_2 * leg_2) / (2 * leg_1 * leg_2);		// D = (x^2 + y^2 - a1^2 - a2^2) / (2 * a1 * a2) �ʱⰪ
	double ang2_def = -1. * acos(D_def);																		// t����2 ���� �ʱⰪ
	double ang1_def = atan2(y_def, x_def) + atan2(leg_2 * sin(-1. * ang2_def), leg_1 + leg_2 * cos(ang2_def));	// t����1 ���� �ʱⰪ
	double D = 0;																								// D = (x^2 + y^2 - a1^2 - a2^2) / (2 * a1 * a2)
	double x_target = 0;																						// �ⱸ x��ǥ ��ǥŸ��
	double x_last_target = 0;																					// �ⱸ x��ǥ �ֽ�Ÿ��
	double y_target = 0;																						// �ⱸ y��ǥ ��ǥŸ��
	double y_last_target = 124;																					// �ⱸ y��ǥ �ֽ�Ÿ��
	double x_temp = 0;																							// ����Ȯ�ο� �ⱸ x��ǥ
	double y_temp = 0;																							// ����Ȯ�ο� �ⱸ y��ǥ
	double x_actual = 0;																						// �ⱸ x��ǥ ������ġ
	double y_actual = 0;																						// �ⱸ y��ǥ ������ġ
	double x_delta = 0;																							// �ⱸ x��ǥ ��ȭ��
	double x_velocity = 0;																						// �ⱸ x��ǥ �ӵ�
	double x_last_velocity = 0;																					// �ⱸ x��ǥ �ֽżӵ�
	double x_velocity_temp = 0;																					// �ⱸ x��ǥ ������ �ӵ�
	double x_acceleration = 0;																					// �ⱸ x��ǥ ���ӵ�
	double x_deceleration = 0;																					// �ⱸ x��ǥ ���ӵ�
	double x_max = 0;																							// �ⱸ x��ǥ �ִ밪
	double x_min = 0;																							// �ⱸ x��ǥ �ּҰ�
};

// �� ��ħ��
class foot {
public:
	DWORD errorCode = 0;					// Error information on the executed function	
	double GARO = 512.6;					// ���α���
	double SERO = 158.8;					// ���α���
	double x_coordinate = 245.84;			// x��ǥ
	double y_coordinate = -68;				// y��ǥ
	double x_target = 0;					// x��ǥ ��ǥŸ��
	double x_last_target = 0;				// x��ǥ �ֽ�Ÿ��
	double y_target = 0;					// y��ǥ ��ǥŸ��
	double y_last_target = 0;				// y��ǥ �ֽ�Ÿ��
	double x_actual = 0;					// ���� x��ǥ
	double y_actual = 0;					// ���� y��ǥ
	double angle_actual = 0;				// ���� ����
	bool home_attained = false;				// Homing Flag	
};
#endif