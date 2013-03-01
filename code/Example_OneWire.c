/**
 *******************************************************************************
 * @file    Example_OneWire.c
 * @author  Vojtěch Vigner <vojtech.vigner@gmail.com>
 * @date    18-February-2013
 * @brief   Simple code example for 1-Wire library.
 *******************************************************************************
 */

#include "OneWire.h"

#define MAX_DEVICES 	8
#define SOME_COMMAND	0xAA

int main(void) {

    uint64_t Addresses[MAX_DEVICES];
    uint64_t iAddress;
    int iCount = 0;
    int i;

    /* Bus initialization */
    OW_Init();

    /* Ready bus for communcation */
    OW_WeakPullUp();

    /* Search for first 1-Wire device */
    iAddress = OW_SearchFirst(0);

    /* Store all device addresses into a array */
    while ((iAddress) && (iCount < MAX_DEVICES)) {
        iCount++;
        Addresses[iCount - 1] = iAddress;
        iAddress = OW_SearchNext();
    }

    if (iCount == 0) {
        printf("No devices found.\r\n");
        return 1;
    }

    /* Reset communication because the last device remained selected */
    OW_Reset();

    for (i = 0; i < iCount; i++) {
        OW_Reset();
        OW_ByteWrite(SOME_COMMAND);
        printf("Device %d response = %d.\r\n", i, OW_ByteRead());
    }

    printf("Finished.\r\n");

    return 0;
}