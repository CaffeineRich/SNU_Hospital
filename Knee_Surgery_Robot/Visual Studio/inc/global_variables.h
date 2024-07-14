#ifndef _GLOBAL_
#define _GLOBAL_
#pragma once
#define PI 3.1415926						// 원주율

// Test 전처리기
#define Test 1
// 1: t/epos모터 구동
// 2: t모터 구동
// 3: epos모터 구동
// 4: 구동X

// Trajectory
class trajectory {
public:
	int cnt_NP = 1;							// Way Point 현재위치
	int number_of_path = 0;                 // Way Point 개수
	bool over_fold;							// Over Fold 구간 Flag
	int flag_move_sector_trajectory = 0;	// Sector Trajectory 실행 Flag
	int flag_move_over_fold_trajectory = 0;	// Over Fold Trajectory 실행 Flag
	int flag_sector_halt = 0;				// Sector Halt Trajectory 실행 Flag
	int flag_over_halt = 0;					// Over Fold Halt Trajectory 실행 Flag
	double division_sec = 0;				// Sector Trajectory 가속구간
	double division_over = 0;				// Over Fold Trajectory 가속구간
	double halt_delay = 0;					// Halt Trajectory 감속구간
	double tmotor_angle_addition = 0;		// t모터1,2 Position 합
};

// 사람 신체
class human_gogwanjeol {
public:
	double x = 0;							// 고관절 x좌표
	double y = 0;							// 고관절 y좌표
};
class human_leg {
public:
	double top = 0;							// 윗다리 길이
	double bottom = 0;						// 아랫다리 길이
	double GARO = 0;						// 다리 가로길이
	double SERO = 0;						// 다리 세로길이
	double angle = 0;						// 무릎~발꿈치와 수평면 사이 각도
};
class human_knee {
public:
	double angle_target = 0;				// 무릎 목표각도
	double angle_last_target = 0;			// 무릎 최신각도
	double angle_max = 0;					// 무릎 최대각도
	double angle_min = 0;					// 무릎 최소각도
	double angle_actual = 0;				// 무릎 실제각도
	double x_target = 0;					// 무릎 x좌표 목표타깃
	double x_last_target = 0;				// 무릎 x좌표 최신타깃
	double y_target = 0;					// 무릎 y좌표 목표타깃
	double y_last_target = 0;				// 무릎 y좌표 최신타깃
	double x_actual = 0;					// 무릎 실제 x좌표
	double y_actual = 0;					// 무릎 실제 y좌표
};

// 허벅지 받침대
class thigh {
public:
	double x_target = 0;					// 허벅지 x좌표 목표타깃
	double x_last_target = 0;				// 허벅지 x좌표 최신타깃
	double y_target = 0;					// 허벅지 y좌표 목표타깃
	double y_last_target = 0;				// 허벅지 y좌표 최신타깃
	double radius = 0;						// 허벅지 반지름(고관절~허벅지 길이)
	double angle_actual = 0;				// 고관절~허벅지와 수평면 사이 실제 각도
	double angle_target = 0;				// 허벅지 각도 목표타깃
	double angle_last_target = 0;			// 허벅지 각도 최신타깃
	double angle_max = 0;					// 허벅지 최대각도
	double angle_min = 0;					// 허벅지 최소각도
	double angle_difference = 0;			// 허벅지 각도 총 변화량
	double velocity = 0;					// 허벅지 속도
	double last_velocity = 0;				// 허벅지 최신속도
	double velocity_temp = 0;				// 허벅지 마지막 속도
	double acceleration = 0;				// 허벅지 가속도
	double deceleration = 0;				// 허벅지 감속도
	double gap_btw_device = 0;				// 허벅지와 기구사이 거리
	double angle_btw_device = 0;			// 허벅지와 기구사이 각도
};
class device {
public:
	const double x_def = 0;																						// 기구 x좌표 초기값
	const double y_def = 124;																					// 기구 y좌표 초기값
	const double leg_1 = 160;																					// a1 = 160mm
	const double leg_2 = 160;																					// a2 = 160mm
	double D_def = (x_def * x_def + y_def * y_def - leg_1 * leg_1 - leg_2 * leg_2) / (2 * leg_1 * leg_2);		// D = (x^2 + y^2 - a1^2 - a2^2) / (2 * a1 * a2) 초기값
	double ang2_def = -1. * acos(D_def);																		// t모터2 각도 초기값
	double ang1_def = atan2(y_def, x_def) + atan2(leg_2 * sin(-1. * ang2_def), leg_1 + leg_2 * cos(ang2_def));	// t모터1 각도 초기값
	double D = 0;																								// D = (x^2 + y^2 - a1^2 - a2^2) / (2 * a1 * a2)
	double x_target = 0;																						// 기구 x좌표 목표타깃
	double x_last_target = 0;																					// 기구 x좌표 최신타깃
	double y_target = 0;																						// 기구 y좌표 목표타깃
	double y_last_target = 124;																					// 기구 y좌표 최신타깃
	double x_temp = 0;																							// 범위확인용 기구 x좌표
	double y_temp = 0;																							// 범위확인용 기구 y좌표
	double x_actual = 0;																						// 기구 x좌표 실제위치
	double y_actual = 0;																						// 기구 y좌표 실제위치
	double x_delta = 0;																							// 기구 x좌표 변화량
	double x_velocity = 0;																						// 기구 x좌표 속도
	double x_last_velocity = 0;																					// 기구 x좌표 최신속도
	double x_velocity_temp = 0;																					// 기구 x좌표 마지막 속도
	double x_acceleration = 0;																					// 기구 x좌표 가속도
	double x_deceleration = 0;																					// 기구 x좌표 감속도
	double x_max = 0;																							// 기구 x좌표 최대값
	double x_min = 0;																							// 기구 x좌표 최소값
};

// 발 받침대
class foot {
public:
	DWORD errorCode = 0;					// Error information on the executed function	
	double GARO = 512.6;					// 가로길이
	double SERO = 158.8;					// 세로길이
	double x_coordinate = 245.84;			// x좌표
	double y_coordinate = -68;				// y좌표
	double x_target = 0;					// x좌표 목표타깃
	double x_last_target = 0;				// x좌표 최신타깃
	double y_target = 0;					// y좌표 목표타깃
	double y_last_target = 0;				// y좌표 최신타깃
	double x_actual = 0;					// 실제 x좌표
	double y_actual = 0;					// 실제 y좌표
	double angle_actual = 0;				// 실제 기울기
	bool home_attained = false;				// Homing Flag	
};
#endif