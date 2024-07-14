#include <stdio.h>
#include <conio.h>
#include "canlib.h"

void Check(const char* id, canStatus stat) {
    if (stat != canOK) {
        char buf[50];
        buf[0] = '\0';
        canGetErrorText(stat, buf, sizeof(buf));
        printf("%s: failed, stat=%d (%s)\n", id, (int)stat, buf);
        exit(1);
    }
}
void dumpMessageLoop(canHandle hnd, int channel_number) {
    canStatus stat = canOK;
    long id;
    unsigned int dlc, flags;
    unsigned char msg[8];
    DWORD timestamp;
    printf("Listening for messages on channel %d, press any key to close\n", channel_number);
    while (!_kbhit()) {
        stat = canReadWait(hnd, &id, msg, &dlc, &flags, &timestamp, 100);
        // Check that the returned status is OK (which means that a message has been received)
        if (stat == canOK) {
            if (flags & canMSG_ERROR_FRAME) {
                printf("***ERROR FRAME RECEIVED***");
            }
            else {
                printf("Id: %ld, DLC: %ld, Msg: %u %u %u %u %u %u %u %u Flags: %lu\n",
                    id, dlc, msg[0], msg[1], msg[2], msg[3], msg[4],
                    msg[5], msg[6], msg[7], timestamp);
            }
        }
        else if (stat != canERR_NOMSG) {
            Check("canRead", stat);
            break;
        }
    }
}
void main(int argc, int* argv[]) {
    canHandle hnd;
    canStatus stat;
    int channel_number = 1;
    canInitializeLibrary();
    printf("Opening channel %d\n", channel_number);
    hnd = canOpenChannel(channel_number, canOPEN_ACCEPT_VIRTUAL);
    if (hnd < 0) {
        Check("canOpenChannel", (canStatus)hnd);
        exit(1);
    }
    printf("Setting bitrate and going bus on\n");
    stat = canSetBusParams(hnd, canBITRATE_1M, 0, 0, 0, 0, 0);
    Check("canSetBusParams", stat);
    stat = canBusOn(hnd);
    Check("canBusOn", stat);
    dumpMessageLoop(hnd, channel_number);
    printf("Going of bus and closing channel");
    stat = canBusOff(hnd);
    Check("canBusOff", stat);
    stat = canClose(hnd);
    Check("canClose", stat);
}