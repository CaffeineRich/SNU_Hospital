#include <iostream>
#include <math.h>
using namespace std;



/*============================================================ 함수 선언============================================================*/
unsigned short crc16(unsigned char* buf, unsigned int len);                                         // CRC
void write_position_velocity_loop(uint8_t msg[], float pos_a, float vel_a, float acc_a);            // 모터 실제 P,V,A 값 -> 모터 Serial 코드값 (모드a)
void read_position(double* pos, uint8_t msg[]);                                                     // 모터 Serial 코드값 -> 모터 실제 P 값 (모드b)
void coordinate_to_angle(double ang[], double x, double y);                                         // xy좌표 -> 모터 실제 P 값 (모드c)
bool validate_coordinate_to_angle(double x, double y);                                              // 기구 좌표가 범위 내에 있는지 확인 (모드c)
void angle_to_coordinate(double coor[], double ang_1, double ang_2);                                // 모터 실제 P 값 -> xy좌표 (모드d)
void send_motor_by_trajectory(double x, double y);                                                  // xy좌표 -> Trajectory Serial Message Send (모드e)



/*============================================================ 전역변수 선언 ============================================================*/
// CRC
const unsigned short crc16_tab[] = { 0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5,
        0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318,
        0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401,
        0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf,
        0xc5ac, 0xd58d, 0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
        0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5,
        0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af,
        0x8948, 0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50,
        0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
        0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f,
        0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4,
        0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a,
        0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
        0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398,
        0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2,
        0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f,
        0xd52c, 0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2,
        0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f,
        0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1,
        0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
        0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f,
        0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45,
        0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba,
        0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

// while문 실행조건
int flag_while = 0;

// 모드 입력
char key = 0;

// 모드a (모터 실제 P,V,A 값 -> 모터 Serial 코드값)
const float pos_min = -360;                                 // P 입력 범위
const float pos_max = 360;
const float vel_min = 50;                                   // V 입력 범위
const float vel_max = 40000;
const float acc_min = 100;                                  // A 입력 범위
const float acc_max = 60000;
float position_actual = 0;                                  // 실제 P 값
float velocity_actual = 0;                                  // 실제 V 값
float acceleration_actual = 0;                              // 실제 A 값
uint8_t msg_send_serial[18] = { 0 };                        // 모터 Serial 코드값

// 모드b (모터 Serial 코드값 -> 모터 실제 P 값)
int start_of_frame = 0x02;                                                                                                              // SOF
int data_length = 0;                                                                                                                    // Data Length
int data_frame = 0;                                                                                                                     // Data Frame
int byte1 = 0, byte2 = 0, byte3 = 0, byte4 = 0;                                                                                         // 모터 P Serial 코드
int crc1_read = 0, crc2_read = 0;                                                                                                       // CRC
int end_of_frame = 0x03;                                                                                                                // EOF
uint8_t msg_read_serial[] = { start_of_frame,data_length,data_frame,byte1, byte2, byte3, byte4, crc1_read, crc2_read, end_of_frame };   // 모터 Serial 코드값
double position = 0;                                                                                                                    // 모터 실제 P 값
double* point_position = &position;                                                                                                     // 모터 실제 P 값 포인터

// 모드c (xy좌표 -> 모터 실제 P 값)
double x_in = 0;                                            // x좌표
double y_in = 0;                                            // y좌표
double angle_out[2] = { 0 };                                // 모터 실제 P 값

// 모드d (모터 실제 P 값 -> xy좌표)
double angle1_in = 0;                                       // 모터1 실제 P 값
double angle2_in = 0;                                       // 모터2 실제 P 값
double coordinate[2] = { 0 };                               // xy좌표

// 모드e (xy좌표 -> Trajectory Serial Message Send)
int number_of_path = 100;                                   // Way Point 개수
double x_delta = 0;                                         // delta x = 목표위치x - 최신타깃x
double y_delta = 0;                                         // delta y = 목표위치y - 최신타깃y
double x_value_const = 0;                                   // x 고정 변화량
double y_value_const = 0;                                   // y 고정 변화량
int k_velocity = 25000;                                     // 모터1,2 속도 상수
float motor1_velocity = 5000;                               // 모터1 속도
float motor2_velocity = 5000;                               // 모터2 속도
float motor_acceleration = 30000;                           // 모터1,2 가속도
double x_last_target = 0;                                   // 최신타깃 x좌표
double y_last_target = 160;                                 // 최신타깃 y좌표
double degree1_last_target = 0;                             // 모터1 최신각도
double degree2_last_target = 0;                             // 모터2 최신각도
double angle_target[2] = { 0 };                             // 모터1,2 목표각도
uint8_t motor1_msg_send_serial[18] = { 0 };                 // 모터1 Serial 코드값
uint8_t motor2_msg_send_serial[18] = { 0 };                 // 모터2 Serial 코드값

// 모드f (Data Length = 1 CRC 값 확인)
uint8_t buf_len_1[] = { data_frame };
unsigned short cyclic_redundancy_check = 0;         // CRC 값 확인

// 모드g (Data Length = 2 CRC 값 확인)
uint8_t buf_len_2[] = { data_frame, byte1 };

// 모드h (Data Length = 5 CRC 값 확인)
int byte5 = 0;
uint8_t buf_len_5[] = { data_frame, byte1, byte2, byte3, byte4 };

// 모드i (Data Length = 13 CRC 값 확인)
int byte6 = 0, byte7 = 0, byte8 = 0, byte9 = 0, byte10 = 0, byte11 = 0, byte12 = 0;
uint8_t buf_len_13[] = { data_frame, byte1, byte2, byte3, byte4, byte5, byte6, byte7, byte8, byte9, byte10, byte11, byte12 };



int main()
{
    while (flag_while == 0)
    {
        // 모드
        cout << "==========수행할 작업을 선택하세요==========\n\n"     // 모드 소개
            << "a: PVA -> Serial\n"
            << "b: Serial -> P\n\n"
            << "c: xy좌표(mm) -> P\n"
            << "d: P -> xy좌표\n\n"
            << "e: xy좌표(mm) -> Trajectory Serial Message Send\n"
            << "f: LEN=1 CRC 값  (frame)\n"
            << "g: LEN=2 CRC 값  (frame, byte1)\n"
            << "h: LEN=5 CRC 값  (frame, byte1~4)\n"
            << "i: LEN=13 CRC 값 (frame, byte1~12)\n\n"
            << "s: While문 종료\n\n" << endl;
        cin >> key;                                                    // 모드 선택

        switch (key)
        {

        // 모터 실제 P,V,A 값 -> 모터 Serial 코드값
        case 'a':
            cout << "Position: " << endl;
            cin >> position_actual;                                                                                 // 모터 실제 P 값 입력
            if (position_actual > pos_max or position_actual < pos_min) position_actual = 0;
            cout << "Velocity: " << endl;
            cin >> velocity_actual;                                                                                 // 모터 실제 V 값 입력
            if (velocity_actual > vel_max or velocity_actual < vel_min) velocity_actual = 20000;
            cout << "Acceleration: " << endl;
            cin >> acceleration_actual;                                                                             // 모터 실제 A 값 입력
            if (acceleration_actual > acc_max or acceleration_actual < acc_min) acceleration_actual = 40000;
            write_position_velocity_loop(msg_send_serial, position_actual, velocity_actual, acceleration_actual);   // 함수 호출
            printf("모터 Serial 코드 = [ ");
            for (int i = 0; i < 18; i++) {
                printf("%02X ", msg_send_serial[i]);                                                                // 모터 Serial 코드값 출력
            }
            printf("]\n\n");
            break;


        // 모터 Serial 코드값 -> 모터 실제 P 값
        case 'b':
            cout << "sof,len,frame,byte1~4,crc_1,crc_2,eof" << endl;
            cin >> std::hex;                                                             // 16진수 입력
            cin >> start_of_frame;                                                       // 모터 Serial 코드값 입력
            cin >> data_length;
            cin >> data_frame;
            cin >> byte1;
            cin >> byte2;
            cin >> byte3;
            cin >> byte4;
            cin >> crc1_read;
            cin >> crc2_read;
            cin >> end_of_frame;
            msg_read_serial[0] = start_of_frame;
            msg_read_serial[1] = data_length;
            msg_read_serial[2] = data_frame;
            msg_read_serial[3] = byte1;
            msg_read_serial[4] = byte2;
            msg_read_serial[5] = byte3;
            msg_read_serial[6] = byte4;
            msg_read_serial[7] = crc1_read;
            msg_read_serial[8] = crc2_read;
            msg_read_serial[9] = end_of_frame;

            read_position(point_position, msg_read_serial);                              // 함수 호출
            cout << "모터 실제 P 값: " << *point_position << " degrees\n\n" << endl;     // 모터 실제 P 값 출력
            break;


        // xy좌표 -> 모터 실제 P 값
        case 'c':
            cout << "xy좌표(mm): " << endl;
            cin >> x_in;                                            // xy좌표 입력
            cin >> y_in;
            if (validate_coordinate_to_angle(x_in, y_in)) {         // 범위 이내 출력 + 함수 호출
                coordinate_to_angle(angle_out, x_in, y_in);         // 함수 호출
                printf("모터 실제 P 값 = [ ");
                for (int i = 0; i < 2; i++) {
                    printf("%f ", angle_out[i]);                    // 모터 실제 P 값 출력
                }
                printf("]\n\n");
            }
            else {
                cout << "Out of Range!!\n\n" << endl;               // 범위 초과 출력
            }
            break;


        // 모터 실제 P 값 -> xy좌표
        case 'd':
            cout << "ID1 모터 각도: " << endl;
            cin >> angle1_in;                                       // 모터1 실제 P 값 입력
            cout << "ID2 모터 각도: " << endl;
            cin >> angle2_in;                                       // 모터2 실제 P 값 입력
            angle_to_coordinate(coordinate, angle1_in, angle2_in);  // 함수 호출
            printf("xy좌표 = [ ");
            for (int i = 0; i < 2; i++) {
                printf("%f ", coordinate[i]);                       // xy좌표 출력
            }
            printf("]\n\n");
            break;


        // xy좌표 -> Trajectory Serial Message Send
        case 'e':
            cout << "xy좌표(mm): " << endl;
            cin >> x_in;                                            // xy좌표 입력
            cin >> y_in;
            if (validate_coordinate_to_angle(x_in, y_in)) {         // 범위 이내 출력 + 함수 호출
                send_motor_by_trajectory(x_in, y_in);               // 함수 호출
            }
            else {
                cout << "Out of Range!!\n\n" << endl;               // 범위 초과 출력
            }
            break;


        // Data Length = 1 CRC 값 확인
        case 'f':
            data_length = 1;                                                     // Data Length = 1
            cout << "frame" << endl;
            cin >> std::hex;                                                     // 16진수 입력
            cin >> data_frame;                                                   // Data Frame 입력
            buf_len_1[0] = data_frame;
            cyclic_redundancy_check = crc16(buf_len_1, data_length);             // 함수 호출
            cout << std::hex;                                                    // 16진수 출력
            cout << "CRC 코드: " << cyclic_redundancy_check << "\n\n" << endl;   // CRC 값 출력
            break;


        // Data Length = 2 CRC 값 확인
        case 'g':
            data_length = 2;                                                      // Data Length = 2
            cout << "frame, byte1" << endl;
            cin >> std::hex;                                                      // 16진수 입력
            cin >> data_frame;                                                    // Data Frame 입력
            cin >> byte1;                                                         // Byte 입력
            buf_len_2[0] = data_frame;                                            // Buffer에 Data Frame 대입
            buf_len_2[1] = byte1;                                                 // Buffer에 Byte 대입
            cyclic_redundancy_check = crc16(buf_len_2, data_length);              // 함수 호출
            cout << std::hex;                                                     // 16진수 출력
            cout << "CRC 코드: " << cyclic_redundancy_check << "\n\n" << endl;    // CRC 값 출력
            break;


        // Data Length = 5 CRC 값 확인
        case 'h':
            data_length = 5;                                                      // Data Length = 5
            cout << "frame, byte1~4" << endl;
            cin >> std::hex;                                                      // 16진수 입력
            cin >> data_frame;                                                    // Data Frame 입력
            cin >> byte1;                                                         // Byte 입력
            cin >> byte2;
            cin >> byte3;
            cin >> byte4;
            buf_len_5[0] = data_frame;                                            // Buffer에 Data Frame 대입
            buf_len_5[1] = byte1;                                                 // Buffer에 Byte 대입
            buf_len_5[2] = byte2;
            buf_len_5[3] = byte3;
            buf_len_5[4] = byte4;
            cyclic_redundancy_check = crc16(buf_len_5, data_length);              // 함수 호출
            cout << std::hex;                                                     // 16진수 출력
            cout << "CRC 코드: " << cyclic_redundancy_check << "\n\n" << endl;    // CRC 값 출력
            break;


        // Data Length = 13 CRC 값 확인
        case 'i':
            data_length = 13;                                                     // Data Length = 13
            cout << "frame, byte1~12" << endl;
            cin >> std::hex;                                                      // 16진수 입력
            cin >> data_frame;                                                    // Data Frame 입력
            cin >> byte1;                                                         // Byte 입력
            cin >> byte2;
            cin >> byte3;
            cin >> byte4;
            cin >> byte5;
            cin >> byte6;
            cin >> byte7;
            cin >> byte8;
            cin >> byte9;
            cin >> byte10;
            cin >> byte11;
            cin >> byte12;
            buf_len_13[0] = data_frame;                                           // Buffer에 Data Frame 대입
            buf_len_13[1] = byte1;                                                // Buffer에 Byte 대입
            buf_len_13[2] = byte2;
            buf_len_13[3] = byte3;
            buf_len_13[4] = byte4;
            buf_len_13[5] = byte5;
            buf_len_13[6] = byte6;
            buf_len_13[7] = byte7;
            buf_len_13[8] = byte8;
            buf_len_13[9] = byte9;
            buf_len_13[10] = byte10;
            buf_len_13[11] = byte11;
            buf_len_13[12] = byte12;
            cyclic_redundancy_check = crc16(buf_len_13, data_length);             // 함수 호출
            cout << std::hex;                                                     // 16진수 출력
            cout << "CRC 코드: " << cyclic_redundancy_check << "\n\n" << endl;    // CRC 값 출력
            break;


        // While문 종료
        case 's':
            flag_while = 1;
            break;
        }

        // 전역변수 초기화
        position_actual = 0;
        velocity_actual = 0;
        acceleration_actual = 0;
        data_length = 0;
        data_frame = 0;
        byte1 = 0, byte2 = 0, byte3 = 0, byte4 = 0, byte5 = 0, byte6 = 0, byte7 = 0, byte8 = 0, byte9 = 0, byte10 = 0, byte11 = 0, byte12 = 0;
        crc1_read = 0, crc2_read = 0;
        position = 0;
    }
    return 0;
}



/*============================================================ 함수 선언============================================================*/
// CRC
unsigned short crc16(unsigned char* buf, unsigned int len)
{
    unsigned int i;
    unsigned short cksum = 0;
    for (i = 0; i < len; i++) {
        cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
    };
    return cksum;
}

// 모터 실제 P,V,A 값 -> 모터 Serial 코드값 (모드a)
void write_position_velocity_loop(uint8_t msg[], float pos_a, float vel_a, float acc_a)
{
    // Data Length, Data Frame
    const uint8_t len = 13;
    const uint8_t frame = 0x5B;

    // Data Byte
    uint8_t d[len] = { 0 };

    // Actual position, velocity, acceleration
    uint32_t pos = pos_a * 1000;
    uint32_t vel = vel_a;
    uint32_t acc = acc_a;

    // position
    d[0] = (pos >> 24) & 0xFF;
    d[1] = (pos >> 16) & 0xFF;
    d[2] = (pos >> 8) & 0xFF;
    d[3] = pos & 0xFF;

    // velocity
    d[4] = (vel >> 24) & 0xFF;
    d[5] = (vel >> 16) & 0xFF;
    d[6] = (vel >> 8) & 0xFF;
    d[7] = vel & 0xFF;

    // acceleration
    d[8] = (acc >> 24) & 0xFF;
    d[9] = (acc >> 16) & 0xFF;
    d[10] = (acc >> 8) & 0xFF;
    d[11] = acc & 0xFF;

    // CRC
    unsigned short cksum;
    uint8_t buf[] = { frame,d[0],d[1],d[2],d[3],d[4],d[5],d[6],d[7],d[8],d[9],d[10],d[11] };
    uint8_t crc[2] = { 0 };
    cksum = crc16(buf, len);
    crc[0] = cksum / 256;
    crc[1] = cksum % 256;

    // 모터 Serial 코드
    msg[0] = 0x02;
    msg[1] = len;
    msg[2] = frame;
    for (int i = 0; i < 12; i++) {
        msg[i + 3] = d[i];
    }
    msg[15] = crc[0];
    msg[16] = crc[1];
    msg[17] = 0x03;
}

// 모터 Serial 코드값 -> 모터 실제 P 값 (모드b)
void read_position(double* pos, uint8_t msg[])
{
    uint32_t pos_data = 0;
    pos_data = (msg[3] << 24) + (msg[4] << 16) + (msg[5] << 8) + msg[6];
    *pos = (double)pos_data / 10000;
}

// xy좌표 -> 모터 실제 P 값 (모드c)
void coordinate_to_angle(double ang[], double x, double y)
{
    const double pi = 3.1415926;
    const double x_def = 0;                                                                                                                     // x좌표 초기값
    const double y_def = 160;                                                                                                                   // y좌표 초기값
    const double leg_1 = 160;                                                                                                                   // a1=160mm
    const double leg_2 = 160;                                                                                                                   // a2=160mm
    const double D_def = ((double)pow(x_def, 2) + (double)pow(y_def, 2) - (double)pow(leg_1, 2) - (double)pow(leg_2, 2)) / (2 * leg_1 * leg_2);
    const double ang2_def = acos(D_def);                                                                                                        // 모터2 각도 초기값
    const double ang1_def = atan2(y_def, x_def + 0.000000001) - atan2(leg_2 * sin(ang2_def), leg_1 + leg_2 * cos(ang2_def));                    // 모터1 각도 초기값
    double D = ((double)pow(x, 2) + (double)pow(y, 2) - (double)pow(leg_1, 2) - (double)pow(leg_2, 2)) / (2 * leg_1 * leg_2);
    double ang_2 = acos(D);                                                                                                                     // 모터2 각도 최종값  
    double ang_1 = atan2(y, x + 0.000000001) - atan2(leg_2 * sin(ang_2), leg_1 + leg_2 * cos(ang_2));                                           // 모터1 각도 최종값
    ang[0] = (ang_1 - ang1_def) * 180 / pi;                                                                                                     // 모터1 각도 변화량
    ang[1] = (ang2_def - ang_2) * 180 / pi;                                                                                                     // 모터2 각도 변화량
}

// 기구 좌표가 범위 내에 있는지 확인 (모드c)
bool validate_coordinate_to_angle(double x, double y)
{
    if (y > 159 &&                                                                  // y > 159
        ((double)pow(x, 2) + (double)pow(y, 2)) <= (double)pow(320, 2)) {           // x^2 + y^2 <= 320^2
        return true;                                                                // 범위 이내 반환
    }
    else
        return false;                                                               // 범위 초과 반환
}

// 모터 실제 P 값 -> xy좌표 (모드d)
void angle_to_coordinate(double coor[], double ang_1, double ang_2)
{
    const double pi = 3.1415926;
    const double x_def = 0;                                                                                                                     // x좌표 초기값
    const double y_def = 160;                                                                                                                   // y좌표 초기값
    const double leg_1 = 160;                                                                                                                   // a1=160mm
    const double leg_2 = 160;                                                                                                                   // a2=160mm
    const double D_def = ((double)pow(x_def, 2) + (double)pow(y_def, 2) - (double)pow(leg_1, 2) - (double)pow(leg_2, 2)) / (2 * leg_1 * leg_2);
    const double ang2_def = acos(D_def);                                                                                                        // 모터2 각도 초기값
    const double ang1_def = atan2(y_def, x_def + 0.000000001) - atan2(leg_2 * sin(ang2_def), leg_1 + leg_2 * cos(ang2_def));                    // 모터1 각도 초기값
    coor[0] = leg_1 * cos(ang1_def + ang_1 * pi / 180) + leg_2 * cos(ang1_def + ang_1 * pi / 180 + ang2_def - ang_2 * pi / 180);                // x좌표 최종값
    coor[1] = leg_1 * sin(ang1_def + ang_1 * pi / 180) + leg_2 * sin(ang1_def + ang_1 * pi / 180 + ang2_def - ang_2 * pi / 180);                // y좌표 최종값
}

// xy좌표 -> Trajectory Serial Message Send (모드e)
void send_motor_by_trajectory(double x, double y)
{
    x_delta = x - x_last_target;                                                                                              // delta x = 목표위치x - 최신타깃x
    y_delta = y - y_last_target;                                                                                              // delta y = 목표위치y - 최신타깃y
    x_value_const = x_delta / number_of_path;                                                                                 // x 고정 변화량
    y_value_const = y_delta / number_of_path;                                                                                 // y 고정 변화량

    // 10ms 간격으로 n번 반복
    for (int i = 0; i < number_of_path; i++) {
        x_last_target += x_value_const;                                                                                       // x좌표 누적 최신화
        y_last_target += y_value_const;                                                                                       // y좌표 누적 최신화

        cout << i + 1 << "번째 실행" << endl;                                                                                 // 실행 횟수 출력
        cout << "[ " << x_last_target << ", " << y_last_target << " ]" << endl;                                               // 최신타깃 xy좌표 출력

        coordinate_to_angle(angle_target, x_last_target, y_last_target);                                                      // xy좌표 -> 목표각도1, 목표각도2
        motor1_velocity = k_velocity * abs(angle_target[0] - degree1_last_target);                                            // kv * | 목표각도1 - 최신각도1 |
        motor2_velocity = k_velocity * abs(angle_target[1] - degree2_last_target);                                            // kv * | 목표각도2 - 최신각도2 |

        cout << "모터1: P = " << angle_target[0] << ", V = " << motor1_velocity << ", A = " << motor_acceleration << endl;    // 모터1 PVA 출력
        cout << "모터2: P = " << angle_target[1] << ", V = " << motor2_velocity << ", A = " << motor_acceleration << endl;    // 모터2 PVA 출력

        write_position_velocity_loop(motor1_msg_send_serial, angle_target[0], motor1_velocity, motor_acceleration);           // 모터1 역기구학
        write_position_velocity_loop(motor2_msg_send_serial, angle_target[1], motor2_velocity, motor_acceleration);           // 모터2 역기구학

        printf("모터1 Serial 코드 = [ ");
        for (int i = 0; i < 18; i++) {
            printf("%02X ", motor1_msg_send_serial[i]);                                                                       // 모터1 Serial 코드값 출력
        }
        printf("]\n");

        printf("모터2 Serial 코드 = [ ");
        for (int i = 0; i < 18; i++) {
            printf("%02X ", motor2_msg_send_serial[i]);                                                                       // 모터2 Serial 코드값 출력
        }
        printf("]\n\n");

        degree1_last_target = angle_target[0];                                                                                // 최신각도1 최신화
        degree2_last_target = angle_target[1];                                                                                // 최신각도2 최신화
    }
}