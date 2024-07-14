//-----------------------------------------------------------
// This is a sample program for the Visual Studio 2017 CANlib tutorial.
// It prints a list of connected CAN interfaces.
//
// For further information please refer to the tutorial section of the CANlib documentation.
//-----------------------------------------------------------
#include "canlib.h"
#include "iostream"
// When called, CheckForError will check for and print any error.
// The program is aborted if an error has occured.
void CheckForError(char cmd[50], canStatus stat)
{
    //if stat not ok, print error
    if (stat != canOK)
    {
        char buf[255];
        buf[0] = '\0';
        canGetErrorText(stat, buf, sizeof(buf));
        printf("[%s] %s: failed, stat=%d\n", cmd, buf, (int)stat);
        printf("ABORT (Press any key)");
        getchar();
        abort();
    }
}
// ListChannels prints a list of all connected CAN interfaces.
void ListChannels()
{
    canStatus stat;
    int number_of_channels;
    int device_channel;
    char device_name[255];
    // Get number of channels
    stat = canGetNumberOfChannels(&number_of_channels);
    CheckForError((char*)"canGetNumberOfChannels", stat);
    if (number_of_channels > 0) {
        printf("%s %d %s\n", "Found", number_of_channels, "channels");
    }
    else {
        printf("Could not find any CAN interface.\n");
    }
    // Loop and print all channels
    for (int i = 0; i < number_of_channels; i++) {
        stat = canGetChannelData(i, canCHANNELDATA_DEVDESCR_ASCII, device_name, sizeof(device_name));
        CheckForError((char*)"canGetChannelData", stat);
        stat = canGetChannelData(i, canCHANNELDATA_CHAN_NO_ON_CARD, &device_channel, sizeof(device_channel));
        CheckForError((char*)"canGetChannelData", stat);
        printf("%s %d %s %d\n", "Found channel:", i, device_name, (device_channel + 1));
    }
}
int main()
{
    //canStatus stat;
    canInitializeLibrary();
    printf("%s\n", "CanlibTutorial started");
    ListChannels();

    return 0;
}