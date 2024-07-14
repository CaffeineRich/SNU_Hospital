#include <Thigh_Foot_Robot.h>
#include <SystemMemory.h>
#include <SharedMemory.h>


/*============================================================ 핸들 호출 ============================================================*/

Thigh_Foot_Robot TFR;                       // 허벅지/발 받침대
HANDLE timerQueue = CreateTimerQueue();     // Timer Queue
HANDLE timer1;                              // Timer1


/*============================================================ 함수 호출 ============================================================*/

VOID CALLBACK move_tmotor_by_trajectory(PVOID lpParameter, BOOLEAN TimerOrWaitFired);                   // Trajectory에 따라 t모터 구동
VOID CALLBACK move_eposmotor_by_trajectory(PVOID lpParameter, BOOLEAN TimerOrWaitFired);                // Trajectory에 따라 epos모터 구동


/*============================================================ Main ============================================================*/
int main()
{
    if (TFR.flag_while) {
        CreateTimerQueueTimer(&timer1, timerQueue, move_tmotor_by_trajectory, NULL, 0, 10, 0);          // 10ms마다 Trajectory에 따라 t모터 구동
        CreateTimerQueueTimer(&timer1, timerQueue, move_eposmotor_by_trajectory, NULL, 0, 50, 0);       // 50ms마다 Trajectory에 따라 epos모터 구동
        TFR.reset_and_offset();                                                                         // 초기화 및 오프셋 설정
    }
    while (TFR.flag_while) TFR.while_loop();                                                            // 허벅지/발 받침대 While문 실행
    return 0;
}


/*============================================================ 함수 선언 ============================================================*/

// Trajectory에 따라 t모터 구동
VOID CALLBACK move_tmotor_by_trajectory(PVOID lpParameter, BOOLEAN TimerOrWaitFired)
{
    TFR.move_Offset();                                                                                  // t모터1,2 Offset 실행
    TFR.move_Sector_Trajectory();                                                                       // Sector Trajectory 실행
    TFR.move_Sector_Halt_Trajectory();                                                                  // Sector Halt Trajectory 실행
    TFR.move_Over_Fold_Trajectory();                                                                    // Over Fold Trajectory 실행
    TFR.move_Over_Fold_Halt_Trajectory();                                                               // Over Fold Halt Trajectory 실행
}
// Trajectory에 따라 epos모터 구동
VOID CALLBACK move_eposmotor_by_trajectory(PVOID lpParameter, BOOLEAN TimerOrWaitFired)
{
    TFR.move_Both_Trajectory();                                                                         // epos모터1,2 Trajectory 실행
}
