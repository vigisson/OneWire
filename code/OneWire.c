/**
 *******************************************************************************
 * @file    OneWire.c
 * @author  Vojtěch Vigner <vojtech.vigner@gmail.com>
 * @version V1.0.5
 * @date    12-February-2013
 * @brief   Provides 1-Wire bus support for STM32Fxxx devices.
 * 
 * @section info Additional Information
 *          This library provides functions to manage the following 
 *          functionalities of the 1-Wire bus from MAXIM:           
 *              - Initialization and configuration.
 *              - Low level communication functions.
 *              - 1-Wire specific CRC calculation.
 *              - Device address operations.
 *              - Parasite powered device support.
 *              - 1-Wire advanced device search, based on MAXIM App. Note 126.
 * 
 *          Library requires one USART for communication with 1-Wire devices. 
 *          Current implementation used Half Duplex USART mode. This means that
 *          only one pin is used for communication.  
 *          
 *          Currently supports and has been tested on STM32F2xx and STM32F4xx
 *          devices. 
 * 
 * @section howto How to use this library
 *          1. Modify hardware specific section in OneWire.h file according to
 *          your HW. Decide if you will be using parasite powered device/s and
 *          enable or disable this support.
 *  
 *          2. Initialize bus using OW_Init().
 * 
 *          3. Now you can use all communication functions. 
 * 
 *          4. See Example_OneWire.c for simple example. 
 *  
 * @copyright The BSD 3-Clause License. 
 * 
 * @section License
 *          Copyright (c) 2013, Vojtěch Vigner <vojtech.vigner@gmail.com> 
 *           
 *          All rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without 
 *          modification, are permitted provided that the following conditions
 *          are met:
 *              - Redistributions of source code must retain the above 
 *                copyright notice, this list of conditions and the following 
 *                disclaimer.
 *              - Redistributions in binary form must reproduce the above 
 *                copyright notice, this list of conditions and the following
 *                disclaimer in the documentation and/or other materials
 *                provided with the distribution.
 *              - The name of the contributors can not be used to endorse or 
 *                promote products derived from this software without specific
 *                prior written permission.
 *
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *          "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *          LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *          FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
 *          COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *          INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *          BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *          LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 *          CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
 *          LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
 *          ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 *          POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 */

#include "OneWire.h"
#include "stm32f407_precise_board.h"

/* Bus specific commands */
#define OW_ROM_READ         0x33
#define OW_ROM_MATCH        0x55
#define OW_ROM_SKIP         0xCC
#define OW_ROM_SEARCH       0xF0
#define OW_ALARM_SEARCH     0xEC

/* Private defines */
#define OW_R                0xF0
#define OW_0                0x00
#define OW_1                0xFF

/* Search related variables */
static struct {
    uint8_t iLastDeviceFlag;
    uint8_t iLastDiscrepancy;
    uint8_t iLastFamilyDiscrepancy;
    uint64_t ROM;
} stSearch;

/* Backup of BRR register for different communication speeds*/
static uint16_t iUSART9600;
static uint16_t iUSART115200;

/* CRC calculation table */
static uint8_t CRCTable[] = {
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};

/**
 * Hardware initialization.
 */
void OW_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStructure;

    /* Enable clock for periphetials */
    OW_GPIO_TX_CLOCK();

#ifndef OW_USE_SINGLE_PIN
    OW_GPIO_RX_CLOCK();
#endif

    OW_USART_CLOCK();

    /* Alternate function config on TX pin */
    GPIO_PinAFConfig(OW_TX_PIN_PORT, OW_TX_PIN_SOURCE, OW_USART_AF);

    /* TX pin configuration */
    GPIO_InitStruct.GPIO_Pin = OW_TX_PIN_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(OW_TX_PIN_PORT, &GPIO_InitStruct);

#ifndef OW_USE_SINGLE_PIN 
    /* Alternate function config on RX pin */
    GPIO_PinAFConfig(OW_RX_PIN_PORT, OW_RX_PIN_SOURCE, OW_USART_AF);
    GPIO_InitStruct.GPIO_Pin = OW_RX_PIN_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(OW_RX_PIN_PORT, &GPIO_InitStruct);
#endif

    /* USART configuration */
    USART_StructInit(&USART_InitStructure);
    
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(OW_USART, &USART_InitStructure);

    /* BRR register backup for 115200 Baud */
    iUSART115200 = OW_USART->BRR;

    /* BRR register backup for 9600 Baud */
    USART_InitStructure.USART_BaudRate = 9600;
    USART_Init(OW_USART, &USART_InitStructure);
    iUSART9600 = OW_USART->BRR;

#ifdef OW_USE_SINGLE_PIN 
    /* Half duplex enable, for single pin communication */
    USART_HalfDuplexCmd(OW_USART, ENABLE);
#endif

    /* USART enable */
    USART_Cmd(OW_USART, ENABLE);
}

/**
 * Read one bit.
 * @return 0 or 1.
 */
uint8_t OW_BitRead(void) {
    /* Make sure that all communication is done and receive buffer is cleared */
    while (USART_GetFlagStatus(OW_USART, USART_FLAG_TC) == RESET);
    while (USART_GetFlagStatus(OW_USART, USART_FLAG_RXNE) == SET)
        USART_ReceiveData(OW_USART);

    /* Send byte */
    USART_SendData(OW_USART, OW_1);

    /* Wait for response */
    while (USART_GetFlagStatus(OW_USART, USART_FLAG_TC) == RESET);

    /* Receive data */
    if (USART_ReceiveData(OW_USART) != OW_1) return 0;

    return 1;
}

/**
 * Read one byte.
 * @return Received byte.
 */
uint8_t OW_ByteRead(void) {
    int i;
    uint8_t iRet = 0;

    /* Read 8 bits */
    for (i = 0; i < 8; i++) {
        if (OW_BitRead()) iRet |= (1 << i);
    }

    return iRet;
}

/**
 * Write one bit.
 * @param bBit 0 or 1.
 */
void OW_BitWrite(const uint8_t bBit) {
    uint8_t bData = OW_0;

    if (bBit) bData = OW_1;

    /* Make sure that all communication is done */
    while (USART_GetFlagStatus(OW_USART, USART_FLAG_RXNE) == SET)
        USART_ReceiveData(OW_USART);
    while (USART_GetFlagStatus(OW_USART, USART_FLAG_TC) == RESET);

    /* Send byte */
    USART_SendData(OW_USART, bData);
}

/**
 * Write one byte.
 * @param bByte Byte to be transmited.
 */
void OW_ByteWrite(const uint8_t bByte) {
    uint8_t i;

    /* Write 8 bits */
    for (i = 0; i < 8; i++) {
        OW_BitWrite(bByte & (1 << i));
    }
}

/**
 * Set RX/TX pin into strong pull-up state.
 */
void OW_StrongPullUp(void) {
#ifdef OW_USE_PARASITE_POWER

#ifdef OW_USE_SINGLE_PIN 
    USART_HalfDuplexCmd(OW_USART, DISABLE);
#endif

    GPIO_SetBits(OW_TX_PIN_PORT, OW_TX_PIN_PIN);
    OW_TX_PIN_PORT->OTYPER &= ~(OW_TX_PIN_PIN);

#endif
}

/**
 * Set RX/TX pin into weak pull-up state.
 */
void OW_WeakPullUp(void) {
#ifdef OW_USE_PARASITE_POWER

    GPIO_SetBits(OW_TX_PIN_PORT, OW_TX_PIN_PIN);
    OW_TX_PIN_PORT->OTYPER |= OW_TX_PIN_PIN;

#ifdef OW_USE_SINGLE_PIN 
    USART_HalfDuplexCmd(OW_USART, ENABLE);
#endif

#endif
}

/**
 * Communication reset and device presence detection.
 * @return OW_OK if device found or OW_NO_DEV if not.
 */
OW_State OW_Reset(void) {
    uint8_t iPresence;

    /* Set USART baudrate to 9600 Baud */
    OW_USART->BRR = iUSART9600;

    /* Make sure that all communication is done and receive buffer is cleared */
    USART_ClearFlag(OW_USART, USART_FLAG_TC);
    while (USART_GetFlagStatus(OW_USART, USART_FLAG_RXNE) == SET)
        USART_ReceiveData(OW_USART);

    /* Write special byte on USART */
    USART_SendData(OW_USART, OW_R);
    while (USART_GetFlagStatus(OW_USART, USART_FLAG_TC) == RESET);

    /* Receive data from USART */
    iPresence = USART_ReceiveData(OW_USART);

    /* Set USART baudrate to 115200 Baud */
    OW_USART->BRR = iUSART115200;

    /* If received data is equal to data transmitted means that there in no
     device present on the bus. Return value equal to 0 means bus error. */
    if ((iPresence != OW_R) && ((iPresence != 0x00))) return OW_OK;

    return OW_NO_DEV;
}

/**
 * Setup the search to skip the current device type on the next call of 
 * OW_SearchNext function.
 */
void OW_FamilySkipSetup(void) {
    /* Set the last discrepancy to last family discrepancy */
    stSearch.iLastDiscrepancy = stSearch.iLastFamilyDiscrepancy;
    stSearch.iLastFamilyDiscrepancy = 0;

    /* Check for end of list */
    if (stSearch.iLastDiscrepancy == 0) stSearch.iLastDeviceFlag = 1;
}

/**
 * Calculate the 1-Wire specific CRC.
 * @param iCRC Input CRC value.
 * @param iValue Value to be added to CRC.
 * @return Resulting CRC value.
 */
uint8_t OW_CRCCalculate(uint8_t iCRC, uint8_t iValue) {
    return CRCTable[iCRC ^ iValue];
}

/**
 * Find the 'first' devices on the 1-Wire bus.
 * @param iFamilyCode Select family code filter or 0 for all. 
 * @return 64-bit device address or 0 if no device found.
 */
uint64_t OW_SearchFirst(uint8_t iFamilyCode) {
    if (iFamilyCode) {
        stSearch.ROM = (uint64_t) iFamilyCode;

        stSearch.iLastDiscrepancy = 64;
        stSearch.iLastFamilyDiscrepancy = 0;
        stSearch.iLastDeviceFlag = 1;
    } else {
        stSearch.ROM = 0;
        stSearch.iLastDiscrepancy = 0;
        stSearch.iLastDeviceFlag = 0;
        stSearch.iLastFamilyDiscrepancy = 0;
    }

    return OW_SearchNext();
}

/**
 * Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing
 * search state.
 * @return 64-bit device address or 0 if no device found.
 */
uint64_t OW_SearchNext(void) {
    uint8_t iSearchDirection;
    int iIDBit, iCmpIDBit;

    /* Initialize for search */
    uint8_t iROMByteMask = 1;
    uint8_t iCRC = 0;
    int iIDBitNumber = 1;
    int iLastZero = 0;
    int iROMByteNumber = 0;
    int iSearchResult = 0;

    /* If the last call was not the last one */
    if (!stSearch.iLastDeviceFlag) {
        /* 1-Wire reset */
        if (OW_Reset() == OW_NO_DEV) {
            /* Reset the search */
            stSearch.iLastDiscrepancy = 0;
            stSearch.iLastDeviceFlag = 0;
            stSearch.iLastFamilyDiscrepancy = 0;
            return 0;
        }



        /* Issue the search command */
        OW_ByteWrite(OW_ROM_SEARCH);

        /* Loop to do the search */
        do {

#ifdef OW_USE_PARASITE_POWER
#warning "FixMe: Charging IC, should work without this."
            OW_StrongPullUp();
            __IO int i;
            for (i = 0; i < 0xFFFF; i++);
            OW_WeakPullUp();
#endif

            /* Read a bit and its complement */
            iIDBit = OW_BitRead();
            iCmpIDBit = OW_BitRead();

            /* Check for no devices on 1-wire */
            if ((iIDBit == 1) && (iCmpIDBit == 1))
                break;
            else {
                /* All devices coupled have 0 or 1 */
                if (iIDBit != iCmpIDBit)
                    /* Bit write value for search */
                    iSearchDirection = iIDBit;
                else {
                    /* if this discrepancy if before the Last Discrepancy
                    on a previous next then pick the same as last time */
                    if (iIDBitNumber < stSearch.iLastDiscrepancy)
                        iSearchDirection = ((((uint8_t*) & stSearch.ROM)[iROMByteNumber] & iROMByteMask) > 0);
                    else
                        /* If equal to last pick 1, if not then pick 0 */
                        iSearchDirection = (iIDBitNumber == stSearch.iLastDiscrepancy);

                    /* If 0 was picked then record its position in iLastZero */
                    if (iSearchDirection == 0) {
                        iLastZero = iIDBitNumber;

                        /* Check for Last discrepancy in family */
                        if (iLastZero < 9)
                            stSearch.iLastFamilyDiscrepancy = iLastZero;
                    }
                }

                /* Set or clear the bit in the ROM byte with mask rom_byte_mask */
                if (iSearchDirection == 1)
                    ((uint8_t*) & stSearch.ROM)[iROMByteNumber] |= iROMByteMask;
                else
                    ((uint8_t*) & stSearch.ROM)[iROMByteNumber] &= ~iROMByteMask;

                /* Set serial number search direction */
                OW_BitWrite(iSearchDirection);

                /* Increment the byte counter and shift the mask */
                iIDBitNumber++;
                iROMByteMask <<= 1;

                /* If the mask is 0 then go to new ROM byte number and reset mask */
                if (iROMByteMask == 0) {
                    /* Accumulate the CRC */
                    iCRC = OW_CRCCalculate(iCRC, ((uint8_t*) & stSearch.ROM)[iROMByteNumber]);
                    iROMByteNumber++;
                    iROMByteMask = 1;
                }
            }
        } while (iROMByteNumber < 8); /* Loop until through all ROM bytes 0-7 */

        /* If the search was successful then */
        if (!((iIDBitNumber < 65) || (iCRC != 0))) {
            stSearch.iLastDiscrepancy = iLastZero;

            /* Check for last device */
            if (stSearch.iLastDiscrepancy == 0)
                stSearch.iLastDeviceFlag = 1;

            iSearchResult = 1;
        }
    }

    /* If no device found then reset counters so next 'search' will be like a first */
    if (!iSearchResult || !((uint8_t*) & stSearch.ROM)[0]) {
        stSearch.iLastDiscrepancy = 0;
        stSearch.iLastDeviceFlag = 0;
        stSearch.iLastFamilyDiscrepancy = 0;
        return 0;
    }

    return stSearch.ROM;
}

/**
 * Read ROM address of device, works only for one device on the bus.
 * @return 64-bit device address.
 */
uint64_t OW_ROMRead(void) {
    uint64_t iRes = 0;
    int i;

    if (OW_Reset() == OW_NO_DEV) return 0;

    OW_ByteWrite(OW_ROM_READ);
    for (i = 0; i < 8; i++)
        ((uint8_t*) & iRes)[i] = OW_ByteRead();

    return iRes;
}

/**
 * Issue ROM match command.
 * @param iAddress 64-bit device address.
 * @return OW_OK if device is present or OW_NO_DEV if not.
 */
OW_State OW_ROMMatch(uint64_t iAddress) {
    int i;

    if (iAddress == OW_ADDRESS_ALL) return OW_ROMSkip();

    if (OW_Reset() == OW_NO_DEV) return OW_NO_DEV;

    OW_ByteWrite(OW_ROM_MATCH);
    for (i = 0; i < 8; i++)
        OW_ByteWrite(((uint8_t*) & iAddress)[i]);

    return OW_OK;
}

/**
 * Issue ROM skip command.
 * @return OW_OK if some device is present or OW_NO_DEV if not.
 */
OW_State OW_ROMSkip(void) {
    if (OW_Reset() == OW_NO_DEV) return OW_NO_DEV;

    OW_ByteWrite(OW_ROM_SKIP);

    return OW_OK;
}