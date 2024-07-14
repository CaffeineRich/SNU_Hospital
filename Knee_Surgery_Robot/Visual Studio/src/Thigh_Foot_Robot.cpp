#include <Thigh_Foot_Robot.h>
using namespace std;


/*============================================================ 변수 호출 ============================================================*/

// Trajectory
trajectory T;

// 사람 신체
human_leg hl;
human_knee hk;
human_gogwanjeol ggj;

// 허벅지 받침대
thigh th;
device dev;
T_Motor tm[2] = { tmotor1_default, tmotor2_default };

// 발 받침대
foot f;
EPOS_Motor em[2] = { eposmotor1_default, eposmotor2_default };


/*============================================================ 함수 선언 ============================================================*/

// 허벅지/발 받침대 생성자
Thigh_Foot_Robot::Thigh_Foot_Robot()
{
#if (Test == 1 || Test == 2)
    TPCANStatus res;
    res = CAN_Initialize(PcanHandle, BIRRATE);
    if (res != PCAN_ERROR_OK) {
        cout << "Can not initialize. Please check the defines in the code.\n";
        return;
    }
    else {
        cout << "Successfully initialized.\n";
        CAN_Transmit_buf->ID = 0x321;
        CAN_Transmit_buf->LEN = 8;
        CAN_Receive_buf->LEN = 8;
        CAN_Data_reset();                           // CAN통신 버퍼 초기화
        CAN_Open_Thread();                          // 무릎각도 입력대기 쓰레드 실행
    }
#else
    CAN_Open_Thread();
#endif
#if (Test == 1 || Test == 3)
    foot_setup();                                   // 발받침대 기본설정 및 초기화
    Homing_Mode();                                  // epos모터1,2 Homing Mode
#else
    flag_while = 1;
#endif
    receive_body_info();                            // 신체정보 수신
}

// CAN통신 버퍼 초기화
void Thigh_Foot_Robot::CAN_Data_reset()
{
    CAN_Reset(PcanHandle);
}
// 무릎각도 입력대기 쓰레드
void Thigh_Foot_Robot::CAN_Thread()
{
    while (CAN_threadFlag) {
        if (flag_offset_complete) {                             // 오프셋 설정이 완료된 경우
            if (GetAsyncKeyState(VK_UP) & 0x8000) {             // 위쪽 화살표 = 허벅지 각도 증가
                angle_input = th.angle_max;
                new_angle_input = true;
                knee_up = true;
                knee_down = false;
                check_current_value = false;
                check_last_target = false;
            }
            else if (GetAsyncKeyState(VK_DOWN) & 0x8000) {      // 아래쪽 화살표 = 허벅지 각도 감소
                angle_input = th.angle_min;
                new_angle_input = true;
                knee_up = false;
                knee_down = true;
                check_current_value = false;
                check_last_target = false;
            }
            else if (GetAsyncKeyState(VK_LEFT) & 0x8000) {      // 왼쪽 화살표 = 현재값 확인
                check_current_value = true;
                new_angle_input = false;
                knee_up = false;
                knee_down = false;
                check_last_target = false;
            }
            else if (GetAsyncKeyState(VK_RIGHT) & 0x8000) {     // 오른쪽 화살표 = 최신타깃 확인
                check_last_target = true;
                new_angle_input = false;
                knee_up = false;
                knee_down = false;
                check_current_value = false;
            }
            else {                                              // 아무것도 안 눌린 경우
                new_angle_input = false;
                knee_up = false;
                knee_down = false;
                check_current_value = false;
                check_last_target = false;
            }
        }
    }
}
// 쓰레드 실행
void Thigh_Foot_Robot::CAN_Open_Thread()
{
    CAN_threadFlag = true;
    _t = new std::thread(&Thigh_Foot_Robot::CAN_Thread, this);
}
// 쓰레드 종료
void Thigh_Foot_Robot::CAN_Close_Thread()
{
    CAN_threadFlag = false;
    _t->detach();
}
// CAN통신을 통해 데이터 수신
void Thigh_Foot_Robot::CAN_Data_Read()
{
    CAN_Read(PcanHandle, CAN_Receive_buf, &CANTimeStamp);
}
// CAN통신을 통해 데이터 송신
void Thigh_Foot_Robot::CAN_Send()
{
    TPCANStatus res;
    res = CAN_Write(PcanHandle, &CAN_Transmit_buf[0]);
    if (res != PCAN_ERROR_OK) {
        std::cout << "Data Send Fail.\n";
        system("PAUSE");
        return;
    }
}

// 발받침대 기본설정 및 초기화
void Thigh_Foot_Robot::foot_setup()
{
    em[0].new_keyHandle = VCS_OpenDevice(em[0].deviceName, em[0].ProtocolStackName, em[0].interfaceName, em[0].portName, &f.errorCode);
    em[0].keyHandle = em[0].new_keyHandle;                                                                          // epos1 핸들 최신화
    if (em[0].keyHandle != 0) {
        cout << "Successfully Opened Device" << endl;
        VCS_ClearFault(em[0].keyHandle, em[0].NodeId, &f.errorCode);                                                // epos1 State = Fault -> Disable
        VCS_SetState(em[0].keyHandle, em[0].NodeId, 0, &f.errorCode);                                               // epos1 State = Disable
        em[1].new_keyHandle = VCS_OpenSubDevice(em[0].keyHandle, em[1].deviceName, em[1].ProtocolStackName, &f.errorCode);
        em[1].keyHandle = em[1].new_keyHandle;                                                                      // epos2 핸들 최신화
        if (em[1].keyHandle != 0) {
            cout << "Successfully Opened Sub_Device" << endl;
            VCS_ClearFault(em[1].keyHandle, em[1].NodeId, &f.errorCode);                                            // epos2 State = Fault -> Disable
            VCS_SetState(em[1].keyHandle, em[1].NodeId, 0, &f.errorCode);                                           // epos2 State = Disable
            if (VCS_SetMotorType(em[0].keyHandle, em[0].NodeId, em[0].MotorType, &f.errorCode)) {                   // epos모터1 타입 선정(DC)
                cout << "Motor1 Setup Succeed" << endl;
                VCS_SetDcMotorParameter(em[0].keyHandle, em[0].NodeId, em[0].NominalCurrent,                        // epos모터1 파라미터 설정
                    em[0].MaxOutputCurrent, em[0].ThermalTimeConstant, &f.errorCode);
                VCS_SetState(em[0].keyHandle, em[0].NodeId, 1, &f.errorCode);                                       // epos1 State = Enable
            }
            else cout << "Motor1 Setup Failed" << endl;                                                             // epos모터1 타입 선정 실패

            if (VCS_SetMotorType(em[1].keyHandle, em[1].NodeId, em[1].MotorType, &f.errorCode)) {                   // epos모터2 타입 선정(EC)
                cout << "Motor2 Setup Succeed" << endl;
                VCS_SetEcMotorParameter(em[1].keyHandle, em[1].NodeId, em[1].NominalCurrent,                        // epos모터2 파라미터 설정
                    em[1].MaxOutputCurrent, em[1].ThermalTimeConstant, em[1].NbOfPolePairs, &f.errorCode);
                VCS_SetState(em[1].keyHandle, em[1].NodeId, 1, &f.errorCode);                                       // epos2 State = Enable
            }
            else cout << "Motor2 Setup Failed" << endl;                                                             // epos모터2 타입 선정 실패
        }
        else cout << "Failed to Open Sub_Device" << endl;                                                           // Open epos2 실패
    }
    else {                                                                                                          // Open epos1 실패
        cout << "Failed to Open Device" << endl;
        return;                                                                                                     // 프로그램 종료
    }
}
// epos모터1,2 Homing Mode
void Thigh_Foot_Robot::Homing_Mode()
{
    VCS_ActivateHomingMode(em[0].keyHandle, em[0].NodeId, &f.errorCode);                                            // epos모터1 Homing 활성화
    VCS_ActivateHomingMode(em[1].keyHandle, em[1].NodeId, &f.errorCode);                                            // epos모터2 Homing 활성화
    VCS_SetHomingParameter(em[0].keyHandle, em[0].NodeId, em[0].HomingAcceleration, em[0].SpeedSwitch,              // epos모터1 Homing 파리미터 설정
        em[0].SpeedIndex, em[0].HomeOffset, em[0].CurrentThreshold, em[0].HomePosition, &f.errorCode);
    VCS_SetHomingParameter(em[1].keyHandle, em[1].NodeId, em[1].HomingAcceleration, em[1].SpeedSwitch,              // epos모터2 Homing 파리미터 설정
        em[1].SpeedIndex, em[1].HomeOffset, em[1].CurrentThreshold, em[1].HomePosition, &f.errorCode);
    VCS_FindHome(em[0].keyHandle, em[0].NodeId, em[0].HomingMethod, &f.errorCode);                                  // epos모터1 Home 탐색
    VCS_FindHome(em[1].keyHandle, em[1].NodeId, em[1].HomingMethod, &f.errorCode);                                  // epos모터2 Home 탐색
    VCS_WaitForHomingAttained(em[0].keyHandle, em[0].NodeId, em[0].Timeout, &f.errorCode);                          // epos모터1 Home 탐색 대기
    VCS_WaitForHomingAttained(em[1].keyHandle, em[1].NodeId, em[1].Timeout, &f.errorCode);                          // epos모터2 Home 탐색 대기
    VCS_GetHomingState(em[0].keyHandle, em[0].NodeId, &em[0].HomingAttained, &em[0].HomingError, &f.errorCode);     // epos모터1 Home 도달 확인
    VCS_GetHomingState(em[1].keyHandle, em[1].NodeId, &em[1].HomingAttained, &em[1].HomingError, &f.errorCode);     // epos모터2 Home 도달 확인
    if (em[0].HomingAttained && em[1].HomingAttained) {                                                             // epos모터1,2 Home 도달 성공
        f.home_attained = true;
        flag_while = 1;                                                                                             // while문 실행
        cout << "Homing Attained" << endl;
        Profile_Position_Mode();                                                                                    // epos모터1,2 Profile Position Mode
    }
    else {                                                                                                          // epos모터1,2 Home 도달 실패
        cout << "Homing Failed" << endl;
        return;
    }
}
// epos모터1,2 Profile Position Mode
void Thigh_Foot_Robot::Profile_Position_Mode()
{
    VCS_SetPositionProfile(em[0].keyHandle, em[0].NodeId, em[0].ProfileVelocity, em[0].ProfileAcceleration,         // epos모터1 PPM 파라미터 설정
        em[0].ProfileDeceleration, &f.errorCode);
    VCS_SetPositionProfile(em[1].keyHandle, em[1].NodeId, em[1].ProfileVelocity, em[1].ProfileAcceleration,         // epos모터2 PPM 파라미터 설정
        em[1].ProfileDeceleration, &f.errorCode);
    VCS_ActivateProfilePositionMode(em[0].keyHandle, em[0].NodeId, &f.errorCode);                                   // epos모터1 PPM 활성화
    VCS_ActivateProfilePositionMode(em[1].keyHandle, em[1].NodeId, &f.errorCode);                                   // epos모터2 PPM 활성화
}
// 신체정보 수신
void Thigh_Foot_Robot::receive_body_info()
{
    hl.top = 480;                                                               // 윗다리 길이
    hl.bottom = 480;                                                            // 아랫다리 길이
    hl.SERO = 80;                                                               // 다리 세로길이
    hl.angle = asin(hl.SERO / hl.bottom) * 180. / PI;                           // 무릎~발꿈치와 수평면 사이 각도
    hl.GARO = hl.top + hl.bottom * cos(hl.angle * PI / 180.);                   // 다리 가로길이

    ggj.x = f.x_coordinate + f.GARO - hl.GARO;                                  // 고관절 x좌표
    ggj.y = f.y_coordinate + f.SERO + hl.SERO;                                  // 고관절 y좌표
    printf("윗다리 길이:   %3.3lf mm\n", hl.top);
    printf("아랫다리 길이: %3.3lf mm\n", hl.bottom);
    printf("다리 가로길이: %3.3lf mm\n", hl.GARO);
    printf("다리 세로길이: %3.3lf mm\n", hl.SERO);
    printf("고관절(x,y):   ( %3.3lf, %3.3lf )\n", ggj.x, ggj.y);

    CAN_Transmit_buf->DATA[0] = 0xF0;
    CAN_Transmit_buf->DATA[1] = 0xF0;
    CAN_Transmit_buf->DATA[2] = 0x00;
    CAN_Transmit_buf->DATA[3] = 0x00;
    CAN_Transmit_buf->DATA[4] = 0x00;
    CAN_Transmit_buf->DATA[5] = 0x00;
    CAN_Transmit_buf->DATA[6] = 0x00;
    CAN_Transmit_buf->DATA[7] = 0x00;
#if (Test == 1 || Test == 2)
    for (int i = 0; i < 500; i++) {
        CAN_Send();                                                             // STM32로 신체정보 수신완료 신호 송신
    }
#else
    printf("[ %02X %02X %02X %02X %02X %02X %02X %02X ]\n", CAN_Transmit_buf->DATA[0], CAN_Transmit_buf->DATA[1], CAN_Transmit_buf->DATA[2], CAN_Transmit_buf->DATA[3], CAN_Transmit_buf->DATA[4], CAN_Transmit_buf->DATA[5], CAN_Transmit_buf->DATA[6], CAN_Transmit_buf->DATA[7]);
    cout << "신체정보 수신완료" << endl;
#endif
}

// 초기화 및 오프셋 설정
void Thigh_Foot_Robot::reset_and_offset()
{
    // Halt Trajectory 감속구간 설정
    T.halt_delay = 35;

    // Sector Trajectory 가속구간, t모터 속도/가속도 설정
    if (480 <= hl.top) {
        T.division_sec = 10. - 0.2333 * (hl.top - 480.);
        tm[0].velocity = 40000.;
        tm[1].velocity = tm[0].velocity;
        tm[0].acceleration = 50000. + 233.33 * (hl.top - 480.);
        tm[1].acceleration = tm[0].acceleration;
    }
    else if (450 <= hl.top && hl.top < 480) {
        T.division_sec = 10;
        tm[0].velocity = 40000.;
        tm[1].velocity = tm[0].velocity;
        tm[0].acceleration = 50000. + 166.67 * (hl.top - 480.);
        tm[1].acceleration = tm[0].acceleration;
    }
    else if (440 <= hl.top && hl.top < 450) {
        T.division_sec = 10;
        tm[0].velocity = 4000. + 3600. * (hl.top - 440.);
        tm[1].velocity = 10000. + 3000. * (hl.top - 440.);
        tm[0].acceleration = 18000. + 2700. * (hl.top - 440.);
        tm[1].acceleration = 30000. + 1500. * (hl.top - 440.);
    }
    else if (430 <= hl.top && hl.top < 440) {
        T.division_sec = 10;
        tm[0].velocity = 4000.;
        tm[1].velocity = 7000. + 300. * (hl.top - 430.);
        tm[0].acceleration = 18000.;
        tm[1].acceleration = 27000. + 300. * (hl.top - 430.);
    }
    else if (420 <= hl.top && hl.top < 430) {
        T.division_sec = 10;
        tm[0].velocity = 4000.;
        tm[1].velocity = 3500. + 350. * (hl.top - 420.);
        tm[0].acceleration = 18000.;
        tm[1].acceleration = 22000. + 500. * (hl.top - 420.);
    }
    else if (hl.top < 420) {
        T.division_sec = 10;
        tm[0].velocity = 4000.;
        tm[1].velocity = 4000.;
        tm[0].acceleration = 18000.;
        tm[1].acceleration = 18000.;
    }

    tm[0].velocity_temp = (uint32_t)round(tm[0].velocity / 1000.);
    tm[1].velocity_temp = (uint32_t)round(tm[1].velocity / 1000.);
    tm[0].acceleration_temp = (uint32_t)round(tm[0].acceleration / 1000.);
    tm[1].acceleration_temp = (uint32_t)round(tm[1].acceleration / 1000.);

    cout << "Trajectory 가속구간: " << T.division_sec << endl;
    cout << "t모터1 Velocity: " << tm[0].velocity_temp * 1000 << endl;
    cout << "t모터2 Velocity: " << tm[1].velocity_temp * 1000 << endl;
    cout << "t모터1 Acceleration: " << tm[0].acceleration_temp * 1000 << endl;
    cout << "t모터2 Acceleration: " << tm[1].acceleration_temp * 1000 << endl;

    th.radius = 0.4 * hl.top;                                                                               // 허벅지 반지름 = 고관절~허벅지 길이
    th.gap_btw_device = ggj.y - dev.y_last_target;                                                          // 허벅지와 기구사이 거리
    th.angle_btw_device = atan2(th.gap_btw_device, th.radius);                                              // 허벅지와 기구사이 각도
    th.x_last_target = ggj.x + th.radius;                                                                   // 허벅지 x좌표 최신타깃
    th.y_last_target = ggj.y;                                                                               // 허벅지 y좌표 최신타깃
    th.angle_last_target = atan2(th.y_last_target - ggj.y, th.x_last_target - ggj.x) * 180. / PI;           // 허벅지 각도 최신타깃
    hk.x_last_target = ggj.x + hl.top * cos(th.angle_last_target * PI / 180.);                              // 무릎 x좌표 최신타깃
    hk.y_last_target = ggj.y + hl.top * sin(th.angle_last_target * PI / 180.);                              // 무릎 y좌표 최신타깃
    f.x_last_target = f.x_coordinate + f.GARO - em[0].last_target;                                          // 발받침 x좌표 최신타깃
    f.y_last_target = f.y_coordinate + f.SERO;                                                              // 발받침 y좌표 최신타깃
    em[0].last_target = em[0].low_limit;                                                                    // epos모터1 최신타깃
    em[1].last_target = acos((f.x_last_target - hk.x_last_target) / hl.bottom) * 180. / PI;                 // epos모터2 최신타깃
    em[0].offset_position = em[0].last_target;                                                              // epos모터1 오프셋위치
    em[1].offset_position = em[1].last_target;                                                              // epos모터2 오프셋위치
    em[1].low_limit = em[1].offset_position;                                                                // epos모터2 하한
    hk.angle_last_target = 180. - th.angle_last_target - em[1].last_target + hl.angle;                      // 무릎 각도 최신타깃
    hk.angle_max = hk.angle_last_target;                                                                    // 무릎 최대각도
#if (Test == 4)
    cout << "무릎 각도: " << hk.angle_max << endl;
    cout << "허벅지 각도: " << th.angle_last_target << endl;
    cout << "기구 좌표: ( " << dev.x_last_target << ", " << dev.y_last_target << " )" << endl;
    cout << "em[0].last_target: " << em[0].last_target << endl;
    cout << "em[1].last_target: " << em[1].last_target << "\n" << endl;
#endif
    th.angle_min = th.angle_last_target;                                                                    // 허벅지 최소각도
    th.angle_max = th.angle_min;                                                                            // 허벅지 최대각도
    while (1) {
        th.angle_max++;
        hk.x_target = ggj.x + hl.top * cos(th.angle_max * PI / 180.);
        hk.y_target = ggj.y + hl.top * sin(th.angle_max * PI / 180.);
        f.y_target = f.y_last_target;
        f.x_target = hk.x_target + sqrt(hl.bottom * hl.bottom - (f.y_target - hk.y_target) * (f.y_target - hk.y_target));
        em[0].target = f.x_coordinate + f.GARO - f.x_target;
        em[1].target = acos((f.x_target - hk.x_target) / hl.bottom) * 180. / PI;
        hk.angle_target = 180. - th.angle_max - em[1].target + hl.angle;
        th.x_target = ggj.x + th.radius * cos(th.angle_max * PI / 180.);
        th.y_target = ggj.y + th.radius * sin(th.angle_max * PI / 180.);
        dev.x_temp = th.x_target + th.gap_btw_device * sin(th.angle_max * PI / 180.);
        dev.y_temp = th.y_target - th.gap_btw_device * cos(th.angle_max * PI / 180.);
#if (Test == 4)
        cout << "무릎 각도: " << hk.angle_target << endl;
        cout << "허벅지 각도: " << th.angle_max << endl;
        cout << "기구 좌표: ( " << dev.x_temp << ", " << dev.y_temp << " )" << endl;
        cout << "허벅지 좌표: ( " << th.x_target << ", " << th.y_target << " )" << endl;
        cout << "em[0].target: " << em[0].target << endl;
        cout << "em[1].target: " << em[1].target << "\n" << endl;
#endif
        if ((validate_thigh_range(dev.x_temp, dev.y_temp) && validate_foot_range(em[0].target, em[1].target)) == false) {
            th.angle_max--;
            hk.x_target = ggj.x + hl.top * cos(th.angle_max * PI / 180.);
            hk.y_target = ggj.y + hl.top * sin(th.angle_max * PI / 180.);
            f.y_target = f.y_last_target;
            f.x_target = hk.x_target + sqrt(hl.bottom * hl.bottom - (f.y_target - hk.y_target) * (f.y_target - hk.y_target));
            em[0].target = f.x_coordinate + f.GARO - f.x_target;
            em[1].target = acos((f.x_target - hk.x_target) / hl.bottom) * 180. / PI;
            hk.angle_target = 180. - th.angle_max - em[1].target + hl.angle;
            th.x_target = ggj.x + th.radius * cos(th.angle_max * PI / 180.);
            th.y_target = ggj.y + th.radius * sin(th.angle_max * PI / 180.);
            dev.x_temp = th.x_target + th.gap_btw_device * sin(th.angle_max * PI / 180.);
            dev.y_temp = th.y_target - th.gap_btw_device * cos(th.angle_max * PI / 180.);
            break;
        }
        else hk.angle_min = hk.angle_target;
    }
    em[1].max_limit = em[1].target;                                                                         // epos모터 구동상한
    dev.x_max = dev.x_temp;                                                                                 // 기구 x좌표 최대값
    dev.x_min = dev.x_temp;                                                                                 // 기구 x좌표 최소값
    while (dev.x_min > ggj.x + th.gap_btw_device) {
        if (validate_thigh_range(dev.x_min, dev.y_temp)) dev.x_min--;
        else {
            dev.x_min++;
            break;
        }
    }
    cout << "허벅지 최대각도: " << th.angle_max << endl;
    cout << "허벅지 최소각도: " << th.angle_min << endl;
    cout << "무릎 최대각도: " << hk.angle_max << endl;
    cout << "무릎 최소각도: " << hk.angle_min << endl;
    cout << "기구 x좌표 최대값: " << dev.x_max << endl;
    cout << "기구 x좌표 최소값: " << dev.x_min << endl;

    th.angle_target = th.angle_last_target;                                                                 // 허벅지 각도 목표타깃 = 허벅지 각도 최신타깃
    hk.x_target = hk.x_last_target;                                                                         // 무릎 x좌표 목표타깃 = 무릎 x좌표 최신타깃
    hk.y_target = hk.y_last_target;                                                                         // 무릎 y좌표 목표타깃 = 무릎 y좌표 최신타깃
    f.x_target = f.x_last_target;                                                                           // 발받침 x좌표 목표타깃 = 발받침 x좌표 최신타깃
    f.y_target = f.y_last_target;                                                                           // 발받침 y좌표 목표타깃 = 발받침 y좌표 최신타깃
    em[0].target = em[0].last_target;                                                                       // epos모터1 최신타깃 = epos모터1 목표타깃
    em[1].target = em[1].last_target;                                                                       // epos모터2 최신타깃 = epos모터2 목표타깃
    th.x_target = th.x_last_target;                                                                         // 허벅지 x좌표 최신타깃 = 허벅지 x좌표 목표타깃
    th.y_target = th.y_last_target;                                                                         // 허벅지 y좌표 최신타깃 = 허벅지 y좌표 목표타깃
    hk.angle_target = hk.angle_last_target;                                                                 // 무릎 각도 목표타깃 = 무릎 각도 최신타깃

    // 오프셋 설정
    dev.x_temp = th.x_target + th.gap_btw_device * sin(th.angle_target * PI / 180.);
    dev.y_temp = th.y_target - th.gap_btw_device * cos(th.angle_target * PI / 180.);
    if ((validate_thigh_range(dev.x_temp, dev.y_temp) && validate_foot_range(em[0].target, em[1].target)) == true) {
        dev.x_delta = th.x_target - dev.x_last_target;                                                      // 기구 x좌표 변화량
        T.number_of_path = (int)(4. * abs(dev.x_delta) + 30.);                                              // Way Point 개수 
        dev.x_velocity = (dev.x_delta / T.number_of_path) / (1. - 1. / T.division_sec);                     // 기구 x좌표 속도
        dev.x_acceleration = dev.x_velocity / (T.number_of_path / T.division_sec);                          // 기구 x좌표 가속도
        flag_offset = 1;                                                                                    // t모터1,2 Offset 실행
        move_eposmotor_PPM();                                                                               // epos모터1,2 PPM 구동
    }
    else {
        cout << "오프셋 범위 오류" << endl;
        return;
    }
}
// 허벅지 좌표가 범위 내에 있는지 확인
bool Thigh_Foot_Robot::validate_thigh_range(double x, double y)
{
    if (!(x < -60 && y < 125) && y > 123 && (x * x + y * y) < 320 * 320) return true;
    else return false;
}
// 발받침대 좌표가 범위 내에 있는지 확인
bool Thigh_Foot_Robot::validate_foot_range(double e1, double e2)
{
    if (e1 >= em[0].low_limit && e1 <= em[0].high_limit && e2 >= em[1].low_limit && e2 <= em[1].high_limit) return true;
    else return false;
}
// epos모터1,2 PPM 구동
void Thigh_Foot_Robot::move_eposmotor_PPM()
{
    em[0].TargetPosition = (long)(em[0].target * 2048.);                                                    // epos모터1: mm -> inc
    em[1].TargetPosition = (long)((em[1].target - em[1].offset_position) * 8601.6);                         // epos모터2: 도 -> inc
#if (Test == 1 || Test == 3)
    VCS_MoveToPosition(em[0].keyHandle, em[0].NodeId, em[0].TargetPosition, TRUE, TRUE, &f.errorCode);      // epos모터1 구동
    VCS_MoveToPosition(em[1].keyHandle, em[1].NodeId, em[1].TargetPosition, TRUE, TRUE, &f.errorCode);      // epos모터2 구동
#endif
}

// While문 실행
void Thigh_Foot_Robot::while_loop()
{
#if(Test == 1 || Test == 2)
    knee_actual_value();                                                                                                    // 무릎 실제값
#endif
    // 위/아래쪽 화살표를 누른 경우
    if (new_angle_input == true && button_pressed == false) {
        button_pressed = true;

        if (T.over_fold == false) {                                                                                         // Over Fold 구간X 인 경우
            th.last_velocity = th.velocity_temp;                                                                            // 허벅지 최신속도 = 허벅지 마지막 속도
            th.x_last_target = th.x_target;                                                                                 // 허벅지 x좌표 최신타깃 = 허벅지 x좌표 마지막 타깃
            th.y_last_target = th.y_target;                                                                                 // 허벅지 x좌표 최신타깃 = 허벅지 x좌표 마지막 타깃
            dev.x_last_target = dev.x_target;                                                                               // 기구 x좌표 최신타깃 = 기구 x좌표 마지막 타깃
            dev.y_last_target = dev.y_target;                                                                               // 기구 y좌표 최신타깃 = 기구 y좌표 마지막 타깃
            th.angle_last_target = atan2(th.y_last_target - ggj.y, th.x_last_target - ggj.x) * 180. / PI;                   // 허벅지 각도 최신타깃
            hk.x_last_target = ggj.x + hl.top * cos(th.angle_last_target * PI / 180.);                                      // 무릎 x좌표 최신타깃
            hk.y_last_target = ggj.y + hl.top * sin(th.angle_last_target * PI / 180.);                                      // 무릎 y좌표 최신타깃
            f.x_last_target = f.x_target;                                                                                   // 발받침 x좌표 최신타깃 = 발받침 x좌표 마지막 타깃
            f.y_last_target = f.y_target;                                                                                   // 발받침 y좌표 최신타깃 = 발받침 y좌표 마지막 타깃
            em[0].last_target = em[0].target;                                                                               // epos모터1 최신타깃 = epos모터1 마지막 타깃
            em[1].last_target = em[1].target;                                                                               // epos모터2 최신타깃 = epos모터2 마지막 타깃
            T.flag_sector_halt = 0;                                                                                         // Halt Trajectory 종료

            th.angle_target = angle_input;
            hk.x_target = ggj.x + hl.top * cos(th.angle_target * PI / 180.);
            hk.y_target = ggj.y + hl.top * sin(th.angle_target * PI / 180.);
            f.y_target = f.y_last_target;
            f.x_target = hk.x_target + sqrt(hl.bottom * hl.bottom - (f.y_target - hk.y_target) * (f.y_target - hk.y_target));
            em[0].target = f.x_coordinate + f.GARO - f.x_target;
            em[1].target = acos((f.x_target - hk.x_target) / hl.bottom) * 180. / PI;
            hk.angle_target = 180. - th.angle_target - em[1].target + hl.angle;
            th.x_target = ggj.x + th.radius * cos(th.angle_target * PI / 180.);
            th.y_target = ggj.y + th.radius * sin(th.angle_target * PI / 180.);
            generate_sector_trajectory();
        }

        else if (T.over_fold == true) {                                                                                     // Over Fold 구간O 인 경우
            dev.x_last_target = dev.x_target;                                                                               // 기구 x좌표 최신타깃 = 기구 x좌표 마지막 타깃
            em[1].last_target = em[1].target;                                                                               // epos모터2 최신타깃 = epos모터2 마지막 타깃
            dev.x_last_velocity = 0;                                                                                        // 기구 x좌표 최신속도 = 기구 x좌표 마지막 속도
            em[1].last_velocity = 0;                                                                                        // epos모터2 최신속도 = epos모터2 마지막 속도
            T.flag_over_halt = 0;                                                                                           // Over Fold Trajectory 종료

            if (knee_down == true) {
                dev.x_target = dev.x_max;
                em[1].target = em[1].max_limit;
            }
            else if (knee_up == true) {
                dev.x_target = dev.x_min;
                em[1].target = em[1].low_limit;
            }
            generate_over_fold_trajectory();
        }
    }
    // 위/아래쪽 화살표를 뗀 경우
    else if (new_angle_input == false && button_pressed == true) {
        button_pressed = false;

        if (T.over_fold == false) {                                                                                         // Over Fold 구간X 인 경우
            th.last_velocity = th.velocity_temp;                                                                            // 허벅지 최신속도 = 허벅지 마지막 속도
            th.x_last_target = th.x_target;                                                                                 // 허벅지 x좌표 최신타깃 = 허벅지 x좌표 마지막 타깃
            th.y_last_target = th.y_target;                                                                                 // 허벅지 x좌표 최신타깃 = 허벅지 x좌표 마지막 타깃
            dev.x_last_target = dev.x_target;                                                                               // 기구 x좌표 최신타깃 = 기구 x좌표 마지막 타깃
            dev.y_last_target = dev.y_target;                                                                               // 기구 y좌표 최신타깃 = 기구 y좌표 마지막 타깃
            th.angle_last_target = atan2(th.y_last_target - ggj.y, th.x_last_target - ggj.x) * 180. / PI;                   // 허벅지 각도 최신타깃
            hk.x_last_target = ggj.x + hl.top * cos(th.angle_last_target * PI / 180.);                                      // 무릎 x좌표 최신타깃
            hk.y_last_target = ggj.y + hl.top * sin(th.angle_last_target * PI / 180.);                                      // 무릎 y좌표 최신타깃
            f.x_last_target = f.x_target;                                                                                   // 발받침 x좌표 최신타깃 = 발받침 x좌표 마지막 타깃
            f.y_last_target = f.y_target;                                                                                   // 발받침 y좌표 최신타깃 = 발받침 y좌표 마지막 타깃
            em[0].last_target = em[0].target;                                                                               // epos모터1 최신타깃 = epos모터1 마지막 타깃
            em[1].last_target = em[1].target;                                                                               // epos모터2 최신타깃 = epos모터2 마지막 타깃
            T.flag_move_sector_trajectory = 0;                                                                              // Sector Trajectory 종료
            T.cnt_NP = 1;                                                                                                   // Way Point 현재위치 초기화

            if (th.last_velocity > 0) T.halt_delay = 2. * (th.angle_max - th.angle_last_target) / th.last_velocity;
            else if (th.last_velocity < 0) T.halt_delay = 2. * (th.angle_min - th.angle_last_target) / th.last_velocity;
            else T.halt_delay = 1;

            if (T.halt_delay < 35) T.halt_delay = 1;
            else T.halt_delay = 35;
            T.flag_sector_halt = 1;
        }

        else if (T.over_fold == true) {                                                                                     // Over Fold 구간O 인 경우
            dev.x_last_target = dev.x_target;                                                                               // 기구 x좌표 최신타깃 = 기구 x좌표 마지막 타깃
            em[1].last_target = em[1].target;                                                                               // epos모터2 최신타깃 = epos모터2 마지막 타깃
            dev.x_last_velocity = dev.x_velocity_temp;                                                                      // 기구 x좌표 최신속도 = 기구 x좌표 마지막 속도
            em[1].last_velocity = em[1].velocity_temp;                                                                      // epos모터2 최신속도 = epos모터2 마지막 속도
            T.flag_move_over_fold_trajectory = 0;                                                                           // Over Fold Trajectory 종료
            T.cnt_NP = 1;                                                                                                   // Way Point 현재위치 초기화

            if (dev.x_last_velocity < 0) T.halt_delay = 2. * (dev.x_min - dev.x_last_target) / dev.x_last_velocity;
            else if (dev.x_last_velocity > 0) T.halt_delay = 2. * (dev.x_max - dev.x_last_target) / dev.x_last_velocity;
            else T.halt_delay = 1;

            if (T.halt_delay < 35) T.halt_delay = 1;
            else T.halt_delay = 35;
            T.flag_over_halt = 1;
        }
    }
    // 왼쪽 화살표를 누른 경우
    else if (check_current_value == true) {
#if(Test == 1)
        if (T.over_fold == false) {
            printf("무릎 실제좌표:             ( %lf, %lf )\n", hk.x_actual, hk.y_actual);
            printf("무릎 실제각도:             %lf 도\n", hk.angle_actual);
        }
        printf("허벅지 받침대 실제좌표:    ( %lf, %lf )\n", dev.x_actual, dev.y_actual);
        printf("epos모터1,2 실제값:        ( %lf, %lf )\n\n", em[0].actual_position / 2048., em[1].actual_position / 8601.6);
#endif
    }
    // 오른쪽 화살표를 누른 경우
    else if (check_last_target == true) {
        if (T.over_fold == false) {
            printf("무릎 마지막 좌표:          ( %lf, %lf )\n", hk.x_last_target, hk.y_last_target);
            printf("무릎 마지막 각도:          %lf 도\n", 180. - th.angle_last_target - em[1].last_target + hl.angle);
        }
        printf("허벅지 받침대 마지막 좌표: ( %lf, %lf )\n", dev.x_last_target, dev.y_last_target);
        printf("epos모터1,2 마지막 타깃:   ( %lf, %lf )\n\n", em[0].last_target, em[1].last_target);
    }
}
// 무릎 실제값
void Thigh_Foot_Robot::knee_actual_value()
{
    CAN_Data_Read();                                                                                    // STM32로부터 CAN 메시지 수신
    read_motor_actual_position();                                                                       // t/epos모터1,2 실제 Position 수신
    forward_kinematics();                                                                               // t모터1,2 실제 Position -> 무릎 실제값
}
// t/epos모터1,2 실제 Position 수신
void Thigh_Foot_Robot::read_motor_actual_position()
{
    tm[0].actual_position_temp = (CAN_Receive_buf->DATA[0] << 24) + (CAN_Receive_buf->DATA[1] << 16)    // t모터1 실제 Position * 10^4
                                 + (CAN_Receive_buf->DATA[2] << 8) + CAN_Receive_buf->DATA[3];
    tm[1].actual_position_temp = (CAN_Receive_buf->DATA[4] << 24) + (CAN_Receive_buf->DATA[5] << 16)    // t모터2 실제 Position * 10^4
                                 + (CAN_Receive_buf->DATA[6] << 8) + CAN_Receive_buf->DATA[7];
    if (CAN_Receive_buf->DATA[0] == 0xFF) {                                                             // t모터1 실제 Position이 음수인 경우
        tm[0].actual_position = (double)(tm[0].actual_position_temp - 4294967296.) / 10000.;
    }
    else if (CAN_Receive_buf->DATA[0] == 0x00) {                                                        // t모터1 실제 Position이 양수인 경우
        tm[0].actual_position = (double)tm[0].actual_position_temp / 10000.;
    }
    if (CAN_Receive_buf->DATA[4] == 0xFF) {                                                             // t모터2 실제 Position이 음수인 경우
        tm[1].actual_position = (double)(tm[1].actual_position_temp - 4294967296.) / 10000.;
    }
    else if (CAN_Receive_buf->DATA[4] == 0x00) {                                                        // t모터2 실제 Position이 양수인 경우
        tm[1].actual_position = (double)tm[1].actual_position_temp / 10000.;
    }
    VCS_GetPositionIs(em[0].keyHandle, em[0].NodeId, &em[0].actual_position, &f.errorCode);             // epos모터1 실제 Position 수신
    VCS_GetPositionIs(em[1].keyHandle, em[1].NodeId, &em[1].actual_position, &f.errorCode);             // epos모터2 실제 Position 수신
}
// t모터1,2 실제 Position -> 무릎 실제값
void Thigh_Foot_Robot::forward_kinematics()
{
    dev.x_actual = dev.leg_1 * cos(dev.ang1_def - tm[0].actual_position * PI / 180.)                    // 기구 실제 x좌표
                   + dev.leg_2 * cos(dev.ang1_def - tm[0].actual_position * PI / 180.
                   + dev.ang2_def + tm[1].actual_position * PI / 180.);
    dev.y_actual = dev.leg_1 * sin(dev.ang1_def - tm[0].actual_position * PI / 180.)                    // 기구 실제 y좌표
                   + dev.leg_2 * sin(dev.ang1_def - tm[0].actual_position * PI / 180.
                   + dev.ang2_def + tm[1].actual_position * PI / 180.);
    th.angle_actual = atan2(dev.y_actual - ggj.y, dev.x_actual - ggj.x) + th.angle_btw_device;          // 고관절~허벅지와 수평면 사이 실제 각도
    f.x_actual = f.x_coordinate + f.GARO - em[0].actual_position / 2048.;                               // 발받침 실제 x좌표
    f.y_actual = f.y_coordinate + f.SERO;                                                               // 발받침 실제 y좌표
    f.angle_actual = em[1].actual_position / 8601.6 + em[1].offset_position;                            // 발받침 실제 각도
    hk.x_actual = ggj.x + hl.top * cos(th.angle_actual);                                                // 무릎 실제 x좌표
    hk.y_actual = ggj.y + hl.top * sin(th.angle_actual);                                                // 무릎 실제 y좌표
    hk.angle_actual = 180. - th.angle_actual * 180. / PI - f.angle_actual + hl.angle;                   // 무릎 실제 각도
}

// t모터1,2 Offset 실행
void Thigh_Foot_Robot::move_Offset()
{
    if (flag_offset == 1) {
        if (0 < T.cnt_NP && T.cnt_NP <= T.number_of_path / T.division_sec) {
            dev.x_target = dev.x_last_target + 0.5 * dev.x_acceleration * T.cnt_NP * T.cnt_NP;
            dev.y_target = dev.y_last_target;
        }
        else if (T.number_of_path / T.division_sec < T.cnt_NP && T.cnt_NP <= (T.division_sec - 1) * T.number_of_path / T.division_sec) {
            dev.x_target = dev.x_last_target + 0.5 * dev.x_velocity * T.number_of_path / T.division_sec + dev.x_velocity * (T.cnt_NP - T.number_of_path / T.division_sec);
            dev.y_target = dev.y_last_target;
        }
        else if ((T.division_sec - 1) * T.number_of_path / T.division_sec < T.cnt_NP && T.cnt_NP <= T.number_of_path) {
            dev.x_target = dev.x_last_target + dev.x_delta - 0.5 * dev.x_acceleration * (T.number_of_path - T.cnt_NP) * (T.number_of_path - T.cnt_NP);
            dev.y_target = dev.y_last_target;
        }
        dev.D = (dev.x_target * dev.x_target + dev.y_target * dev.y_target - dev.leg_1 * dev.leg_1 - dev.leg_2 * dev.leg_2) / (2 * dev.leg_1 * dev.leg_2);
        tm[1].angle = -1. * acos(dev.D);
        tm[0].angle = atan2(dev.y_target, dev.x_target) + atan2(dev.leg_2 * sin(-1. * tm[1].angle), dev.leg_1 + dev.leg_2 * cos(tm[1].angle));
        tm[0].position = (dev.ang1_def - tm[0].angle) * 180. / PI;                                          // t모터1 Position
        tm[1].position = (tm[1].angle - dev.ang2_def) * 180. / PI;                                          // t모터2 Position
        tm[0].position_temp = (uint32_t)(tm[0].position * 1000.);                                           // t모터1 Position * 10^3
        tm[1].position_temp = (uint32_t)(tm[1].position * 1000.);                                           // t모터2 Position * 10^3

        CAN_Transmit_buf->DATA[0] = (tm[0].position_temp >> 24) & 0xFF;                                     // t모터1 Position * 10^3 Serial 코드
        CAN_Transmit_buf->DATA[1] = (tm[0].position_temp >> 16) & 0xFF;                                     // buf[0~3] (4byte)
        CAN_Transmit_buf->DATA[2] = (tm[0].position_temp >> 8) & 0xFF;
        CAN_Transmit_buf->DATA[3] = tm[0].position_temp & 0xFF;
        CAN_Transmit_buf->DATA[4] = (tm[1].position_temp >> 24) & 0xFF;                                     // t모터2 Position * 10^3 Serial 코드
        CAN_Transmit_buf->DATA[5] = (tm[1].position_temp >> 16) & 0xFF;                                     // buf[4~7] (4byte)
        CAN_Transmit_buf->DATA[6] = (tm[1].position_temp >> 8) & 0xFF;
        CAN_Transmit_buf->DATA[7] = tm[1].position_temp & 0xFF;
#if(Test == 1 || Test == 2)
        CAN_Send();
#else
        printf("t모터1: %2.4lf\t\tt모터2: %2.4lf\n", tm[0].position, tm[1].position);
        //printf("[ %02X %02X %02X %02X %02X %02X %02X %02X ]\n", CAN_Transmit_buf->DATA[0], CAN_Transmit_buf->DATA[1], CAN_Transmit_buf->DATA[2], CAN_Transmit_buf->DATA[3], CAN_Transmit_buf->DATA[4], CAN_Transmit_buf->DATA[5], CAN_Transmit_buf->DATA[6], CAN_Transmit_buf->DATA[7]);
#endif
        tm[0].last_position = tm[0].position;                                                               // t모터1 최신 Position = t모터1 Position
        tm[1].last_position = tm[1].position;                                                               // t모터2 최신 Position = t모터2 Position
        T.cnt_NP++;                                                                                         // 다음 Way Point로 이동

        if (T.cnt_NP > T.number_of_path) {                                                                  // 모든 Way Point를 지난 경우
            cout << "Passed All Way Points" << endl;
            dev.x_last_target = dev.x_target;                                                               // 기구 x좌표 최신타깃 = 기구 x좌표 마지막 타깃
            dev.y_last_target = dev.y_target;                                                               // 기구 y좌표 최신타깃 = 기구 y좌표 마지막 타깃
            flag_offset = 0;                                                                                // Offset 종료
            flag_offset_complete = true;
            T.cnt_NP = 1;                                                                                   // Way Point 현재위치 초기화

            CAN_Transmit_buf->DATA[0] = 0xF0;
            CAN_Transmit_buf->DATA[1] = 0xFF;
            CAN_Transmit_buf->DATA[2] = tm[0].velocity_temp & 0xFF;
            CAN_Transmit_buf->DATA[3] = tm[0].acceleration_temp & 0xFF;
            CAN_Transmit_buf->DATA[4] = tm[1].velocity_temp & 0xFF;
            CAN_Transmit_buf->DATA[5] = tm[1].acceleration_temp & 0xFF;
            CAN_Transmit_buf->DATA[6] = 0x00;
            CAN_Transmit_buf->DATA[7] = 0x00;
#if (Test == 1 || Test == 2)
            for (int i = 0; i < 500; i++) {
                CAN_Send();                                                                                 // STM32로 오프셋 완료 신호 송신
            }
#else
            printf("[ %02X %02X %02X %02X %02X %02X %02X %02X ]\n", CAN_Transmit_buf->DATA[0], CAN_Transmit_buf->DATA[1], CAN_Transmit_buf->DATA[2], CAN_Transmit_buf->DATA[3], CAN_Transmit_buf->DATA[4], CAN_Transmit_buf->DATA[5], CAN_Transmit_buf->DATA[6], CAN_Transmit_buf->DATA[7]);
#endif
        }
    }
}

// Sector Trajectory
void Thigh_Foot_Robot::generate_sector_trajectory()
{
    T.cnt_NP = 1;                                                                                                                       // Way Point 현재위치 초기화
    dev.x_target = th.x_target + th.gap_btw_device * sin(th.angle_target * PI / 180.);                                                  // 기구 x좌표 목표타깃
    dev.y_target = th.y_target - th.gap_btw_device * cos(th.angle_target * PI / 180.);                                                  // 기구 y좌표 목표타깃
    dev.D = (dev.x_target * dev.x_target + dev.y_target * dev.y_target - dev.leg_1 * dev.leg_1 - dev.leg_2 * dev.leg_2) / (2 * dev.leg_1 * dev.leg_2);
    tm[1].angle = -1. * acos(dev.D);
    tm[0].angle = atan2(dev.y_target, dev.x_target) + atan2(dev.leg_2 * sin(-1. * tm[1].angle), dev.leg_1 + dev.leg_2 * cos(tm[1].angle));
    tm[0].position = (dev.ang1_def - tm[0].angle) * 180. / PI;                                                                          // t모터1 Position
    tm[1].position = (tm[1].angle - dev.ang2_def) * 180. / PI;                                                                          // t모터2 Position
    T.tmotor_angle_addition = abs(tm[0].position - tm[0].last_position) + abs(tm[1].position - tm[1].last_position);                    // t모터1,2 Position 합

    // t모터1,2 Position 합에 따라 Way Point 개수 설정
    if (T.tmotor_angle_addition < 5) T.number_of_path = (int)(14. * T.tmotor_angle_addition + 30.);
    else if (5 <= T.tmotor_angle_addition && T.tmotor_angle_addition < 10) T.number_of_path = (int)(10. * (T.tmotor_angle_addition - 5.) + 100.);
    else if (10 <= T.tmotor_angle_addition && T.tmotor_angle_addition < 30) T.number_of_path = (int)(2.5 * (T.tmotor_angle_addition - 10.) + 150.);
    else if (30 <= T.tmotor_angle_addition && T.tmotor_angle_addition < 50) T.number_of_path = (int)(2.5 * (T.tmotor_angle_addition - 30.) + 200.);
    else if (50 <= T.tmotor_angle_addition && T.tmotor_angle_addition < 70) T.number_of_path = (int)(2.5 * (T.tmotor_angle_addition - 50.) + 250.);
    else if (70 <= T.tmotor_angle_addition && T.tmotor_angle_addition < 90) T.number_of_path = (int)(2.5 * (T.tmotor_angle_addition - 70.) + 300.);
    else if (90 <= T.tmotor_angle_addition && T.tmotor_angle_addition < 110) T.number_of_path = (int)(5. * (T.tmotor_angle_addition - 90.) + 350.);
    else if (110 <= T.tmotor_angle_addition && T.tmotor_angle_addition < 130) T.number_of_path = (int)(2.5 * (T.tmotor_angle_addition - 110.) + 450.);
    else if (130 <= T.tmotor_angle_addition && T.tmotor_angle_addition < 150) T.number_of_path = (int)(2.5 * (T.tmotor_angle_addition - 130.) + 500.);
    else if (150 <= T.tmotor_angle_addition) T.number_of_path = (int)(10. * (T.tmotor_angle_addition - 150.) + 550.);

    // 윗다리 길이에 따라 Way Point 개수 설정
    if (480 <= hl.top) T.number_of_path = (int)(T.number_of_path * (1. + 0.0043 * (hl.top - 480.)));
    else if (450 <= hl.top && hl.top < 480) T.number_of_path = T.number_of_path;
    else if (440 <= hl.top && hl.top < 450) T.number_of_path = (int)(T.number_of_path * (1.2 - 0.02 * (hl.top - 440.)));
    else if (430 <= hl.top && hl.top < 440) T.number_of_path = (int)(T.number_of_path * (1.3 - 0.01 * (hl.top - 430.)));
    else if (420 <= hl.top && hl.top < 430) T.number_of_path = (int)(T.number_of_path * (1.4 - 0.01 * (hl.top - 420.)));
    else if (hl.top < 420) T.number_of_path = (int)(T.number_of_path * (1.55 - 0.005 * (hl.top - 390.)));

    th.angle_difference = th.angle_target - th.angle_last_target;                                                                       // 허벅지 각도 총 변화량 = 각도 목표타깃 - 각도 최신타깃
    th.velocity = (th.angle_difference / T.number_of_path - 0.5 * th.last_velocity / T.division_sec) / (1. - 1. / T.division_sec);      // 허벅지 속도
    th.acceleration = (th.velocity - th.last_velocity) / (T.number_of_path / T.division_sec);                                           // 허벅지 가속도
    th.deceleration = th.velocity / (T.number_of_path / T.division_sec);                                                                // 허벅지 감속도

    cout << "number of path = " << T.number_of_path << endl;
    T.flag_move_sector_trajectory = 1;                                                                                                  // Sector Trajectory 실행
}
void Thigh_Foot_Robot::move_Sector_Trajectory()
{
    if (T.flag_move_sector_trajectory == 1 && T.flag_sector_halt == 0) {
        if (0 < T.cnt_NP && T.cnt_NP <= T.number_of_path / T.division_sec) {
            th.angle_target = th.angle_last_target + 0.5 * th.acceleration * T.cnt_NP * T.cnt_NP + th.last_velocity * T.cnt_NP;
            th.velocity_temp = th.acceleration * T.cnt_NP + th.last_velocity;
        }
        else if (T.number_of_path / T.division_sec < T.cnt_NP && T.cnt_NP <= (T.division_sec - 1) * T.number_of_path / T.division_sec) {
            th.angle_target = th.angle_last_target + 0.5 * (th.velocity + th.last_velocity) * T.number_of_path / T.division_sec + th.velocity * (T.cnt_NP - T.number_of_path / T.division_sec);
            th.velocity_temp = th.velocity;
        }
        else if ((T.division_sec - 1) * T.number_of_path / T.division_sec < T.cnt_NP && T.cnt_NP <= T.number_of_path) {
            th.angle_target = th.angle_last_target + th.angle_difference - 0.5 * th.deceleration * (T.number_of_path - T.cnt_NP) * (T.number_of_path - T.cnt_NP);
            th.velocity_temp = th.deceleration * (T.number_of_path - T.cnt_NP);
        }
        th.x_target = ggj.x + th.radius * cos(th.angle_target * PI / 180.);                                 // 기구 x좌표 목표타깃 = 고관절 x좌표 + R * cos(허벅지 각도 목표타깃)
        th.y_target = ggj.y + th.radius * sin(th.angle_target * PI / 180.);                                 // 기구 y좌표 목표타깃 = 고관절 y좌표 + R * sin(허벅지 각도 목표타깃)
        dev.x_target = th.x_target + th.gap_btw_device * sin(th.angle_target * PI / 180.);                  // 기구 x좌표 목표타깃
        dev.y_target = th.y_target - th.gap_btw_device * cos(th.angle_target * PI / 180.);                  // 기구 y좌표 목표타깃
        hk.x_target = ggj.x + hl.top * cos(th.angle_target * PI / 180.);                                    // 무릎 x좌표 목표타깃 = 고관절 x좌표 + 사람 윗다리 길이 * cos(허벅지 각도 목표타깃)
        hk.y_target = ggj.y + hl.top * sin(th.angle_target * PI / 180.);                                    // 무릎 y좌표 목표타깃 = 고관절 y좌표 + 사람 윗다리 길이 * sin(허벅지 각도 목표타깃)
        f.y_target = f.y_last_target;
        f.x_target = hk.x_target + sqrt(hl.bottom * hl.bottom - (hk.y_target - f.y_target) * (hk.y_target - f.y_target));
        em[0].target = f.x_coordinate + f.GARO - f.x_target;                                                // epos모터1 목표타깃
        em[1].target = acos((f.x_target - hk.x_target) / hl.bottom) * 180. / PI;                            // epos모터2 목표타깃
        hk.angle_last_target = 180. - th.angle_target - em[1].target + hl.angle;                            // 무릎 최신각도
        dev.D = (dev.x_target * dev.x_target + dev.y_target * dev.y_target - dev.leg_1 * dev.leg_1 - dev.leg_2 * dev.leg_2) / (2 * dev.leg_1 * dev.leg_2);
        tm[1].angle = -1. * acos(dev.D);
        tm[0].angle = atan2(dev.y_target, dev.x_target) + atan2(dev.leg_2 * sin(-1. * tm[1].angle), dev.leg_1 + dev.leg_2 * cos(tm[1].angle));
        tm[0].position = (dev.ang1_def - tm[0].angle) * 180. / PI;                                          // t모터1 Position
        tm[1].position = (tm[1].angle - dev.ang2_def) * 180. / PI;                                          // t모터2 Position
        tm[0].position_temp = (uint32_t)(tm[0].position * 1000.);                                           // t모터1 Position * 10^3
        tm[1].position_temp = (uint32_t)(tm[1].position * 1000.);                                           // t모터2 Position * 10^3

        CAN_Transmit_buf->DATA[0] = (tm[0].position_temp >> 24) & 0xFF;                                     // t모터1 Position * 10^3 Serial 코드
        CAN_Transmit_buf->DATA[1] = (tm[0].position_temp >> 16) & 0xFF;                                     // buf[0~3] (4byte)
        CAN_Transmit_buf->DATA[2] = (tm[0].position_temp >> 8) & 0xFF;
        CAN_Transmit_buf->DATA[3] = tm[0].position_temp & 0xFF;
        CAN_Transmit_buf->DATA[4] = (tm[1].position_temp >> 24) & 0xFF;                                     // t모터2 Position * 10^3 Serial 코드
        CAN_Transmit_buf->DATA[5] = (tm[1].position_temp >> 16) & 0xFF;                                     // buf[4~7] (4byte)
        CAN_Transmit_buf->DATA[6] = (tm[1].position_temp >> 8) & 0xFF;
        CAN_Transmit_buf->DATA[7] = tm[1].position_temp & 0xFF;
#if(Test == 1 || Test == 2)
        CAN_Send();
#elif(Test == 4)
        printf("허벅지 각도: %2.5lf\t기구 좌표: ( %2.5lf, %2.5lf )\tepos모터1: %2.5lf\tepos모터2: %2.5lf\t무릎각도: %3.5lf\n", th.angle_target, dev.x_target, dev.y_target, em[0].target, em[1].target, hk.angle_last_target);
        //printf("[ %02X %02X %02X %02X %02X %02X %02X %02X ]\n", CAN_Transmit_buf->DATA[0], CAN_Transmit_buf->DATA[1], CAN_Transmit_buf->DATA[2], CAN_Transmit_buf->DATA[3], CAN_Transmit_buf->DATA[4], CAN_Transmit_buf->DATA[5], CAN_Transmit_buf->DATA[6], CAN_Transmit_buf->DATA[7]);
#endif
        tm[0].last_position = tm[0].position;                                                               // t모터1 최신 Position = t모터1 Position
        tm[1].last_position = tm[1].position;                                                               // t모터2 최신 Position = t모터2 Position
        T.cnt_NP++;                                                                                         // 다음 Way Point로 이동

        if (T.cnt_NP > T.number_of_path) {                                                                  // 모든 Way Point를 지난 경우
            cout << "Passed All Way Points" << endl;
            th.last_velocity = 0;                                                                           // 허벅지 최신속도 = 0
            th.x_last_target = th.x_target;                                                                 // 허벅지 x좌표 최신타깃 = 허벅지 x좌표 마지막 타깃
            th.y_last_target = th.y_target;                                                                 // 허벅지 y좌표 최신타깃 = 허벅지 y좌표 마지막 타깃
            dev.x_last_target = dev.x_target;                                                               // 기구 x좌표 최신타깃 = 기구 x좌표 마지막 타깃
            dev.y_last_target = dev.y_target;                                                               // 기구 y좌표 최신타깃 = 기구 y좌표 마지막 타깃
            th.angle_last_target = atan2(th.y_last_target - ggj.y, th.x_last_target - ggj.x) * 180. / PI;   // 허벅지 각도 최신타깃
            f.x_last_target = f.x_target;                                                                   // 발받침 x좌표 최신타깃 = 발받침 x좌표 마지막 타깃
            f.y_last_target = f.y_target;                                                                   // 발받침 y좌표 최신타깃 = 발받침 y좌표 마지막 타깃
            em[0].last_target = em[0].target;                                                               // epos모터1 최신타깃 = epos모터1 마지막 타깃
            em[1].last_target = em[1].target;                                                               // epos모터2 최신타깃 = epos모터2 마지막 타깃
            hk.x_last_target = ggj.x + hl.top * cos(th.angle_last_target * PI / 180.);                      // 무릎 x좌표 최신타깃
            hk.y_last_target = ggj.y + hl.top * sin(th.angle_last_target * PI / 180.);                      // 무릎 y좌표 최신타깃
            T.flag_move_sector_trajectory = 0;                                                              // Sector Trajectory 종료

            if (th.angle_last_target >= th.angle_max - 0.1) {
                CAN_Transmit_buf->DATA[0] = 0xF0;
                CAN_Transmit_buf->DATA[1] = 0xFD;
                CAN_Transmit_buf->DATA[2] = 0x00;
                CAN_Transmit_buf->DATA[3] = 0x00;
                CAN_Transmit_buf->DATA[4] = 0x00;
                CAN_Transmit_buf->DATA[5] = 0x00;
                CAN_Transmit_buf->DATA[6] = 0x00;
                CAN_Transmit_buf->DATA[7] = 0x00;
#if (Test == 1 || Test == 2)
                for (int i = 0; i < 100; i++) {
                    CAN_Send();
                }
#else
                printf("[ %02X %02X %02X %02X %02X %02X %02X %02X ]\n", CAN_Transmit_buf->DATA[0], CAN_Transmit_buf->DATA[1], CAN_Transmit_buf->DATA[2], CAN_Transmit_buf->DATA[3], CAN_Transmit_buf->DATA[4], CAN_Transmit_buf->DATA[5], CAN_Transmit_buf->DATA[6], CAN_Transmit_buf->DATA[7]);
#endif
                T.over_fold = true;                                                                         // Over Fold 구간O
                if (knee_up == true) {                                                                      // 위쪽 화살표가 눌린 경우
                    dev.x_target = dev.x_min;
                    em[1].target = em[1].low_limit;
                    generate_over_fold_trajectory();                                                        // Over Fold Trajectory 생성
                }
            }
        }
    }
}
void Thigh_Foot_Robot::move_Sector_Halt_Trajectory()
{
    if (T.flag_sector_halt == 1 && T.flag_move_sector_trajectory == 0) {
        th.acceleration = -1. * th.last_velocity / T.halt_delay;
        th.angle_target = th.angle_last_target + 0.5 * th.acceleration * T.cnt_NP * T.cnt_NP + th.last_velocity * T.cnt_NP;
        th.velocity_temp = th.last_velocity + th.acceleration * T.cnt_NP;

        th.x_target = ggj.x + th.radius * cos(th.angle_target * PI / 180.);                                 // 기구 x좌표 목표타깃 = 고관절 x좌표 + R * cos(허벅지 각도 목표타깃)
        th.y_target = ggj.y + th.radius * sin(th.angle_target * PI / 180.);                                 // 기구 y좌표 목표타깃 = 고관절 y좌표 + R * sin(허벅지 각도 목표타깃)
        dev.x_target = th.x_target + th.gap_btw_device * sin(th.angle_target * PI / 180.);                  // 기구 x좌표 목표타깃
        dev.y_target = th.y_target - th.gap_btw_device * cos(th.angle_target * PI / 180.);                  // 기구 y좌표 목표타깃
        hk.x_target = ggj.x + hl.top * cos(th.angle_target * PI / 180.);                                    // 무릎 x좌표 목표타깃 = 고관절 x좌표 + 사람 윗다리 길이 * cos(허벅지 각도 목표타깃)
        hk.y_target = ggj.y + hl.top * sin(th.angle_target * PI / 180.);                                    // 무릎 y좌표 목표타깃 = 고관절 y좌표 + 사람 윗다리 길이 * sin(허벅지 각도 목표타깃)
        f.y_target = f.y_last_target;
        f.x_target = hk.x_target + sqrt(hl.bottom * hl.bottom - (hk.y_target - f.y_target) * (hk.y_target - f.y_target));
        em[0].target = f.x_coordinate + f.GARO - f.x_target;                                                // epos모터1 목표타깃
        em[1].target = acos((f.x_target - hk.x_target) / hl.bottom) * 180. / PI;                            // epos모터2 목표타깃
        hk.angle_last_target = 180. - th.angle_target - em[1].target + hl.angle;                            // 무릎 최신각도
        dev.D = (dev.x_target * dev.x_target + dev.y_target * dev.y_target - dev.leg_1 * dev.leg_1 - dev.leg_2 * dev.leg_2) / (2 * dev.leg_1 * dev.leg_2);
        tm[1].angle = -1. * acos(dev.D);
        tm[0].angle = atan2(dev.y_target, dev.x_target) + atan2(dev.leg_2 * sin(-1. * tm[1].angle), dev.leg_1 + dev.leg_2 * cos(tm[1].angle));
        tm[0].position = (dev.ang1_def - tm[0].angle) * 180. / PI;                                          // t모터1 Position
        tm[1].position = (tm[1].angle - dev.ang2_def) * 180. / PI;                                          // t모터2 Position
        tm[0].position_temp = (uint32_t)(tm[0].position * 1000.);                                           // t모터1 Position * 10^3
        tm[1].position_temp = (uint32_t)(tm[1].position * 1000.);                                           // t모터2 Position * 10^3

        CAN_Transmit_buf->DATA[0] = (tm[0].position_temp >> 24) & 0xFF;                                     // t모터1 Position * 10^3 Serial 코드
        CAN_Transmit_buf->DATA[1] = (tm[0].position_temp >> 16) & 0xFF;                                     // buf[0~3] (4byte)
        CAN_Transmit_buf->DATA[2] = (tm[0].position_temp >> 8) & 0xFF;
        CAN_Transmit_buf->DATA[3] = tm[0].position_temp & 0xFF;
        CAN_Transmit_buf->DATA[4] = (tm[1].position_temp >> 24) & 0xFF;                                     // t모터2 Position * 10^3 Serial 코드
        CAN_Transmit_buf->DATA[5] = (tm[1].position_temp >> 16) & 0xFF;                                     // buf[4~7] (4byte)
        CAN_Transmit_buf->DATA[6] = (tm[1].position_temp >> 8) & 0xFF;
        CAN_Transmit_buf->DATA[7] = tm[1].position_temp & 0xFF;
#if(Test == 1 || Test == 2)
        CAN_Send();
#elif(Test == 4)
        printf("허벅지 각도: %2.5lf\t기구 좌표: ( %2.5lf, %2.5lf )\tepos모터1: %2.5lf\tepos모터2: %2.5lf\t무릎각도: %3.5lf\n", th.angle_target, dev.x_target, dev.y_target, em[0].target, em[1].target, hk.angle_last_target);
        //printf("[ %02X %02X %02X %02X %02X %02X %02X %02X ]\n", CAN_Transmit_buf->DATA[0], CAN_Transmit_buf->DATA[1], CAN_Transmit_buf->DATA[2], CAN_Transmit_buf->DATA[3], CAN_Transmit_buf->DATA[4], CAN_Transmit_buf->DATA[5], CAN_Transmit_buf->DATA[6], CAN_Transmit_buf->DATA[7]);
#endif
        tm[0].last_position = tm[0].position;                                                               // t모터1 최신 Position = t모터1 Position
        tm[1].last_position = tm[1].position;                                                               // t모터2 최신 Position = t모터2 Position
        T.cnt_NP++;                                                                                         // 다음 Way Point로 이동

        if (T.cnt_NP > T.halt_delay) {                                                                      // 모든 Way Point를 지난 경우
            cout << "Passed All Way Points" << endl;
            th.last_velocity = 0;                                                                           // 허벅지 최신속도 = 0
            th.x_last_target = th.x_target;                                                                 // 허벅지 x좌표 최신타깃 = 허벅지 x좌표 마지막 타깃
            th.y_last_target = th.y_target;                                                                 // 허벅지 y좌표 최신타깃 = 허벅지 y좌표 마지막 타깃
            dev.x_last_target = dev.x_target;                                                               // 기구 x좌표 최신타깃 = 기구 x좌표 마지막 타깃
            dev.y_last_target = dev.y_target;                                                               // 기구 y좌표 최신타깃 = 기구 y좌표 마지막 타깃
            th.angle_last_target = atan2(th.y_last_target - ggj.y, th.x_last_target - ggj.x) * 180. / PI;   // 허벅지 각도 최신타깃
            f.x_last_target = f.x_target;                                                                   // 발받침 x좌표 최신타깃 = 발받침 x좌표 마지막 타깃
            f.y_last_target = f.y_target;                                                                   // 발받침 y좌표 최신타깃 = 발받침 y좌표 마지막 타깃
            em[0].last_target = em[0].target;                                                               // epos모터1 최신타깃 = epos모터1 마지막 타깃
            em[1].last_target = em[1].target;                                                               // epos모터2 최신타깃 = epos모터2 마지막 타깃
            hk.x_last_target = ggj.x + hl.top * cos(th.angle_last_target * PI / 180.);                      // 무릎 x좌표 최신타깃
            hk.y_last_target = ggj.y + hl.top * sin(th.angle_last_target * PI / 180.);                      // 무릎 y좌표 최신타깃
            T.flag_sector_halt = 0;                                                                         // Sector Halt Trajectory 종료
        }
    }
}

// Over Fold Trajectory
void Thigh_Foot_Robot::generate_over_fold_trajectory()
{
    T.cnt_NP = 1;
    dev.D = (dev.x_target * dev.x_target + dev.y_target * dev.y_target - dev.leg_1 * dev.leg_1 - dev.leg_2 * dev.leg_2) / (2 * dev.leg_1 * dev.leg_2);
    tm[1].angle = -1. * acos(dev.D);
    tm[0].angle = atan2(dev.y_target, dev.x_target) + atan2(dev.leg_2 * sin(-1. * tm[1].angle), dev.leg_1 + dev.leg_2 * cos(tm[1].angle));
    tm[0].position = (dev.ang1_def - tm[0].angle) * 180. / PI;                                                                          // t모터1 Position
    tm[1].position = (tm[1].angle - dev.ang2_def) * 180. / PI;                                                                          // t모터2 Position
    T.tmotor_angle_addition = abs(tm[0].position - tm[0].last_position) + abs(tm[1].position - tm[1].last_position);                    // t모터1,2 Position 합

    T.number_of_path = (int)(7.5 * T.tmotor_angle_addition + 25.);                                                                      // Way Point 개수
    T.division_over = 4;                                                                                                                // Over Fold Trajectory 가속구간

    dev.x_delta = dev.x_target - dev.x_last_target;
    dev.x_velocity = (dev.x_delta / T.number_of_path) / (1. - 1. / T.division_over);                                                    // 기구 x좌표 속도
    dev.x_acceleration = (dev.x_velocity - dev.x_last_velocity) / (T.number_of_path / T.division_over);                                 // 기구 x좌표 가속도
    dev.x_deceleration = dev.x_velocity / (T.number_of_path / T.division_over);                                                         // 기구 x좌표 감속도

    em[1].delta = em[1].target - em[1].last_target;                                                                                     // epos모터2 변화량
    em[1].velocity = (em[1].delta / T.number_of_path) / (1. - 1. / T.division_over);                                                    // epos모터2 속도
    em[1].acceleration = (em[1].velocity - em[1].last_velocity) / (T.number_of_path / T.division_over);                                 // epos모터2 가속도
    em[1].deceleration = em[1].velocity / (T.number_of_path / T.division_over);                                                         // epos모터2 감속도

    cout << "number of path = " << T.number_of_path << endl;
    T.flag_move_over_fold_trajectory = 1;
}
void Thigh_Foot_Robot::move_Over_Fold_Trajectory()
{
    if (T.flag_move_over_fold_trajectory == 1 && T.flag_over_halt == 0) {
        if (0 < T.cnt_NP && T.cnt_NP <= T.number_of_path / T.division_over) {
            dev.x_target = dev.x_last_target + 0.5 * dev.x_acceleration * T.cnt_NP * T.cnt_NP;
            em[1].target = em[1].last_target + 0.5 * em[1].acceleration * T.cnt_NP * T.cnt_NP;
            dev.x_velocity_temp = dev.x_acceleration * T.cnt_NP + dev.x_last_velocity;
            em[1].velocity_temp= em[1].acceleration * T.cnt_NP + em[1].last_velocity;
        }
        else if (T.number_of_path / T.division_over < T.cnt_NP && T.cnt_NP <= (T.division_over - 1) * T.number_of_path / T.division_over) {
            dev.x_target = dev.x_last_target + 0.5 * dev.x_velocity * T.number_of_path / T.division_over + dev.x_velocity * (T.cnt_NP - T.number_of_path / T.division_over);
            em[1].target = em[1].last_target + 0.5 * em[1].velocity * T.number_of_path / T.division_over + em[1].velocity * (T.cnt_NP - T.number_of_path / T.division_over);
            dev.x_velocity_temp = dev.x_velocity;
            em[1].velocity_temp = em[1].velocity;
        }
        else if ((T.division_over - 1) * T.number_of_path / T.division_over < T.cnt_NP && T.cnt_NP <= T.number_of_path) {
            dev.x_target = dev.x_last_target + dev.x_delta - 0.5 * dev.x_acceleration * (T.number_of_path - T.cnt_NP) * (T.number_of_path - T.cnt_NP);
            em[1].target = em[1].last_target + em[1].delta - 0.5 * em[1].acceleration * (T.number_of_path - T.cnt_NP) * (T.number_of_path - T.cnt_NP);
            dev.x_velocity_temp = dev.x_acceleration * (T.number_of_path - T.cnt_NP);
            em[1].velocity_temp = em[1].acceleration * (T.number_of_path - T.cnt_NP);
        }
        dev.D = (dev.x_target * dev.x_target + dev.y_target * dev.y_target - dev.leg_1 * dev.leg_1 - dev.leg_2 * dev.leg_2) / (2 * dev.leg_1 * dev.leg_2);
        tm[1].angle = -1. * acos(dev.D);
        tm[0].angle = atan2(dev.y_target, dev.x_target) + atan2(dev.leg_2 * sin(-1. * tm[1].angle), dev.leg_1 + dev.leg_2 * cos(tm[1].angle));
        tm[0].position = (dev.ang1_def - tm[0].angle) * 180. / PI;                                          // t모터1 Position
        tm[1].position = (tm[1].angle - dev.ang2_def) * 180. / PI;                                          // t모터2 Position
        tm[0].position_temp = (uint32_t)(tm[0].position * 1000.);                                           // t모터1 Position * 10^3
        tm[1].position_temp = (uint32_t)(tm[1].position * 1000.);                                           // t모터2 Position * 10^3

        CAN_Transmit_buf->DATA[0] = (tm[0].position_temp >> 24) & 0xFF;                                     // t모터1 Position * 10^3 Serial 코드
        CAN_Transmit_buf->DATA[1] = (tm[0].position_temp >> 16) & 0xFF;                                     // buf[0~3] (4byte)
        CAN_Transmit_buf->DATA[2] = (tm[0].position_temp >> 8) & 0xFF;
        CAN_Transmit_buf->DATA[3] = tm[0].position_temp & 0xFF;
        CAN_Transmit_buf->DATA[4] = (tm[1].position_temp >> 24) & 0xFF;                                     // t모터2 Position * 10^3 Serial 코드
        CAN_Transmit_buf->DATA[5] = (tm[1].position_temp >> 16) & 0xFF;                                     // buf[4~7] (4byte)
        CAN_Transmit_buf->DATA[6] = (tm[1].position_temp >> 8) & 0xFF;
        CAN_Transmit_buf->DATA[7] = tm[1].position_temp & 0xFF;
#if(Test == 1 || Test == 2)
        CAN_Send();
#elif(Test == 4)
        printf("기구 좌표: ( %2.5lf, %2.5lf )\tepos모터1: %2.5lf\tepos모터2: %2.5lf\n", dev.x_target, dev.y_target, em[0].target, em[1].target);
        printf("t모터1,2: ( %2.5lf, %2.5lf )\n", tm[0].position, tm[1].position);
        //printf("[ %02X %02X %02X %02X %02X %02X %02X %02X ]\n", CAN_Transmit_buf->DATA[0], CAN_Transmit_buf->DATA[1], CAN_Transmit_buf->DATA[2], CAN_Transmit_buf->DATA[3], CAN_Transmit_buf->DATA[4], CAN_Transmit_buf->DATA[5], CAN_Transmit_buf->DATA[6], CAN_Transmit_buf->DATA[7]);
#endif
        tm[0].last_position = tm[0].position;                                                               // t모터1 최신 Position = t모터1 Position
        tm[1].last_position = tm[1].position;                                                               // t모터2 최신 Position = t모터2 Position
        T.cnt_NP++;                                                                                         // 다음 Way Point로 이동

        if (T.cnt_NP > T.number_of_path) {                                                                  // 모든 Way Point를 지난 경우
            cout << "Passed All Way Points" << endl;
            dev.x_last_target = dev.x_target;
            em[1].last_target = em[1].target;
            dev.x_last_velocity = 0;
            em[1].last_velocity = 0;
            T.flag_move_over_fold_trajectory = 0;
            if (dev.x_last_target >= dev.x_max - 0.1) {
                CAN_Transmit_buf->DATA[0] = 0xF0;
                CAN_Transmit_buf->DATA[1] = 0xFE;
                CAN_Transmit_buf->DATA[2] = tm[0].velocity_temp & 0xFF;
                CAN_Transmit_buf->DATA[3] = tm[0].acceleration_temp & 0xFF;
                CAN_Transmit_buf->DATA[4] = tm[1].velocity_temp & 0xFF;
                CAN_Transmit_buf->DATA[5] = tm[1].acceleration_temp & 0xFF;
                CAN_Transmit_buf->DATA[6] = 0x00;
                CAN_Transmit_buf->DATA[7] = 0x00;
#if (Test == 1 || Test == 2)
                for (int i = 0; i < 500; i++) {
                    CAN_Send();
                }
#else
                printf("[ %02X %02X %02X %02X %02X %02X %02X %02X ]\n", CAN_Transmit_buf->DATA[0], CAN_Transmit_buf->DATA[1], CAN_Transmit_buf->DATA[2], CAN_Transmit_buf->DATA[3], CAN_Transmit_buf->DATA[4], CAN_Transmit_buf->DATA[5], CAN_Transmit_buf->DATA[6], CAN_Transmit_buf->DATA[7]);
#endif
                T.over_fold = false;                                                                        // Over Fold 구간X
                if (knee_down == true) {                                                                    // 아래쪽 화살표가 눌린 경우
                    th.angle_target = angle_input;
                    hk.x_target = ggj.x + hl.top * cos(th.angle_target * PI / 180.);
                    hk.y_target = ggj.y + hl.top * sin(th.angle_target * PI / 180.);
                    f.y_target = f.y_last_target;
                    f.x_target = hk.x_target + sqrt(hl.bottom * hl.bottom - (f.y_target - hk.y_target) * (f.y_target - hk.y_target));
                    em[0].target = f.x_coordinate + f.GARO - f.x_target;
                    em[1].target = acos((f.x_target - hk.x_target) / hl.bottom) * 180. / PI;
                    hk.angle_target = 180. - th.angle_target - em[1].target + hl.angle;
                    th.x_target = ggj.x + th.radius * cos(th.angle_target * PI / 180.);
                    th.y_target = ggj.y + th.radius * sin(th.angle_target * PI / 180.);
                    generate_sector_trajectory();                                                           // Sector Trajectory 생성
                }
            }
        }
    }
}
void Thigh_Foot_Robot::move_Over_Fold_Halt_Trajectory()
{
    if (T.flag_over_halt == 1 && T.flag_move_over_fold_trajectory == 0) {
        dev.x_acceleration = -1. * dev.x_last_velocity / T.halt_delay;
        em[1].acceleration = -1. * em[1].last_velocity / T.halt_delay;
        dev.x_target = dev.x_last_target + 0.5 * dev.x_acceleration * T.cnt_NP * T.cnt_NP + dev.x_last_velocity * T.cnt_NP;
        em[1].target = em[1].last_target + 0.5 * em[1].acceleration * T.cnt_NP * T.cnt_NP + em[1].last_velocity * T.cnt_NP;
        dev.x_velocity_temp = dev.x_last_velocity + dev.x_acceleration * T.cnt_NP;
        em[1].velocity_temp = em[1].last_velocity + em[1].acceleration * T.cnt_NP;

        dev.D = (dev.x_target * dev.x_target + dev.y_target * dev.y_target - dev.leg_1 * dev.leg_1 - dev.leg_2 * dev.leg_2) / (2 * dev.leg_1 * dev.leg_2);
        tm[1].angle = -1. * acos(dev.D);
        tm[0].angle = atan2(dev.y_target, dev.x_target) + atan2(dev.leg_2 * sin(-1. * tm[1].angle), dev.leg_1 + dev.leg_2 * cos(tm[1].angle));
        tm[0].position = (dev.ang1_def - tm[0].angle) * 180. / PI;                                          // t모터1 Position
        tm[1].position = (tm[1].angle - dev.ang2_def) * 180. / PI;                                          // t모터2 Position
        tm[0].position_temp = (uint32_t)(tm[0].position * 1000.);                                           // t모터1 Position * 10^3
        tm[1].position_temp = (uint32_t)(tm[1].position * 1000.);                                           // t모터2 Position * 10^3

        CAN_Transmit_buf->DATA[0] = (tm[0].position_temp >> 24) & 0xFF;                                     // t모터1 Position * 10^3 Serial 코드
        CAN_Transmit_buf->DATA[1] = (tm[0].position_temp >> 16) & 0xFF;                                     // buf[0~3] (4byte)
        CAN_Transmit_buf->DATA[2] = (tm[0].position_temp >> 8) & 0xFF;
        CAN_Transmit_buf->DATA[3] = tm[0].position_temp & 0xFF;
        CAN_Transmit_buf->DATA[4] = (tm[1].position_temp >> 24) & 0xFF;                                     // t모터2 Position * 10^3 Serial 코드
        CAN_Transmit_buf->DATA[5] = (tm[1].position_temp >> 16) & 0xFF;                                     // buf[4~7] (4byte)
        CAN_Transmit_buf->DATA[6] = (tm[1].position_temp >> 8) & 0xFF;
        CAN_Transmit_buf->DATA[7] = tm[1].position_temp & 0xFF;
#if(Test == 1 || Test == 2)
        CAN_Send();
#elif(Test == 4)
        printf("기구 좌표: ( %2.5lf, %2.5lf )\tepos모터1: %2.5lf\tepos모터2: %2.5lf\n", dev.x_target, dev.y_target, em[0].target, em[1].target);
        printf("t모터1,2: ( %2.5lf, %2.5lf )\n", tm[0].position, tm[1].position);
        //printf("[ %02X %02X %02X %02X %02X %02X %02X %02X ]\n", CAN_Transmit_buf->DATA[0], CAN_Transmit_buf->DATA[1], CAN_Transmit_buf->DATA[2], CAN_Transmit_buf->DATA[3], CAN_Transmit_buf->DATA[4], CAN_Transmit_buf->DATA[5], CAN_Transmit_buf->DATA[6], CAN_Transmit_buf->DATA[7]);
#endif
        tm[0].last_position = tm[0].position;                                                               // t모터1 최신 Position = t모터1 Position
        tm[1].last_position = tm[1].position;                                                               // t모터2 최신 Position = t모터2 Position
        T.cnt_NP++;                                                                                         // 다음 Way Point로 이동

        if (T.cnt_NP > T.halt_delay) {                                                                      // 모든 Way Point를 지난 경우
            cout << "Passed All Way Points" << endl;
            dev.x_last_target = dev.x_target;
            em[1].last_target = em[1].target;
            dev.x_last_velocity = 0;
            em[1].last_velocity = 0;
            T.flag_over_halt = 0;
        }
    }
}

// epos모터1,2 Trajectory 실행
void Thigh_Foot_Robot::move_Both_Trajectory()
{
    if (T.flag_move_sector_trajectory == 1 || T.flag_move_over_fold_trajectory == 1 ||
        T.flag_sector_halt == 1 || T.flag_over_halt == 1) {
        move_eposmotor_PPM();                                                                               // epos모터1,2 PPM 구동
    }
}
