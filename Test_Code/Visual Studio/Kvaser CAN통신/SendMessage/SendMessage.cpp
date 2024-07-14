#include <stdio.h>
#include "canlib.h"
void Check(const char* id, canStatus stat)
{
    if (stat != canOK) {
        char buf[50];
        buf[0] = '\0';
        canGetErrorText(stat, buf, sizeof(buf));
        printf("%s: failed, stat=%d (%s)\n", id, (int)stat, buf);
    }
}
void main(int argc, char* argv[]) {
    canHandle hnd;
    canStatus stat;
    int channel_number = 0;
    char msg[4] = { 0, 0, 0, 0 };
    canInitializeLibrary();
    printf("Opening channel %d\n", channel_number);
    hnd = canOpenChannel(channel_number, canOPEN_ACCEPT_VIRTUAL);
    if (hnd < 0) {
        Check("canOpenChannel", (canStatus)hnd);
        exit(1);
    }
    printf("Setting bitrate and going bus on\n");
    stat = canSetBusParams(hnd, canBITRATE_1M, 5, 2, 1, 1, 0);
    Check("canSetBusParams", stat);
    stat = canBusOn(hnd);
    Check("canBusOn", stat);
    printf("Writing a message to the channel and waiting for it to be sent \n");
    stat = canWrite(hnd, 1025, msg, 4, 0);
    Check("canWrite", stat);
    stat = canWriteSync(hnd, 1000);
    Check("canWriteSync", stat);
    printf("Going off bus and closing channel");
    stat = canBusOff(hnd);
    Check("canBusOff", stat);
    stat = canClose(hnd);
    Check("canClose", stat);
}