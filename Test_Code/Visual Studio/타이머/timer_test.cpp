#include <iostream>
#include <stdio.h>
#include <windows.h>
using namespace std;

VOID CALLBACK hello(PVOID lpParameter, BOOLEAN TimerOrWaitFired);
VOID CALLBACK nice(PVOID lpParameter, BOOLEAN TimerOrWaitFired);

HANDLE timerQueue_1 = CreateTimerQueue();
HANDLE timerQueue_2 = CreateTimerQueue();
HANDLE timer_1;
HANDLE timer_2;

int main() {

	CreateTimerQueueTimer(&timer_1, timerQueue_1, hello, NULL, 0, 1000, 0);
	CreateTimerQueueTimer(&timer_2, timerQueue_1, nice, NULL, 0, 500, 0);

	while (1) {
		Sleep(5000);
		DeleteTimerQueueEx(timerQueue_1, INVALID_HANDLE_VALUE);
		return 0;
	}
	
}

VOID CALLBACK hello(PVOID lpParameter, BOOLEAN TimerOrWaitFired) {
	printf("hello\n");
}
VOID CALLBACK nice(PVOID lpParameter, BOOLEAN TimerOrWaitFired) {
	printf("nice\n");
}