/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== rfEasyLinkTx.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/mw/display/Display.h>
#include <ti/mw/display/DisplayExt.h>

/* Board Header files */
#include "Board.h"

/* EasyLink API Header files */
#include "easylink/EasyLink.h"

#include <ti/drivers/UART.h>

/***** Defines *****/

/* Undefine to remove address filter and async mode */
#define RFEASYLINKTX_ASYNC
#define RFEASYLINKRX_ASYNC
#define RFEASYLINKRX_ADDR_FILTER

#define RFEASYLINKTX_TASK_STACK_SIZE    1024
#define RFEASYLINKTX_TASK_PRIORITY      2

#define RFEASYLINKEX_TASK_STACK_SIZE    1024
#define RFEASYLINKEX_TASK_PRIORITY      3

#define RFEASYLINKTX_BURST_SIZE         10
#define RFEASYLINKTXPAYLOAD_LENGTH      30

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] = {
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/***** Variable declarations *****/
Task_Struct txTask,rxTask;    /* not static so you can see in ROV */
static Task_Params txTaskParams, rxTaskParams;
static uint8_t txTaskStack[RFEASYLINKTX_TASK_STACK_SIZE];
static uint8_t rxTaskStack[RFEASYLINKEX_TASK_STACK_SIZE];

static Display_Handle hDisplaySerial;
static uint8_t node_indentifier;

/* The RX Output struct contains statistics about the RX operation of the radio */
PIN_Handle pinHandle;

uint16_t result_buf[8];

#ifdef RFEASYLINKRX_ASYNC
static Semaphore_Handle rxDoneSem;
#endif

#ifdef RFEASYLINKTX_ASYNC
static Semaphore_Handle txDoneSem,txSwiSem;
#endif //RFEASYLINKTX_ASYNC

extern ti_sysbios_knl_Clock_Handle clockRfTxn;

/***** Function prototypes ******/
void radioTxFn();

/***** Function definitions *****/
#ifdef RFEASYLINKRX_ASYNC
void rxDoneCb(EasyLink_RxPacket * rxPacket, EasyLink_Status status)
{
//	uint8_t i;
    if (status == EasyLink_Status_Success)
    {
        /* Toggle LED2 to indicate RX */
        PIN_setOutputValue(pinHandle, Board_LED2,!PIN_getOutputValue(Board_LED2));
/*		for(i = 0; i !=8; i++)
		{
			result_buf[i] = (*(uint16_t *)(rxPacket->payload+2*(i+1)));
		}*/

        // received databyte into global variable
        node_indentifier = rxPacket->payload[0];
		Semaphore_post(rxDoneSem);
    }
    else if(status == EasyLink_Status_Aborted)
    {
        /* Toggle LED1 to indicate command aborted */
//        PIN_setOutputValue(pinHandle, Board_LED1,!PIN_getOutputValue(Board_LED1));
    }
    else
    {
        /* Toggle LED1 and LED2 to indicate error */
        PIN_setOutputValue(pinHandle, Board_LED1,!PIN_getOutputValue(Board_LED1));
        PIN_setOutputValue(pinHandle, Board_LED2,!PIN_getOutputValue(Board_LED2));
    }

//    Semaphore_post(rxDoneSem);
}
#endif

#ifdef RFEASYLINKTX_ASYNC
void txDoneCb(EasyLink_Status status)
{
    if (status == EasyLink_Status_Success)
    {
        /* Toggle LED1 to indicate TX */
        PIN_setOutputValue(pinHandle, Board_LED1,!PIN_getOutputValue(Board_LED1));
    }
    else if(status == EasyLink_Status_Aborted)
    {
        /* Toggle LED2 to indicate command aborted */
        PIN_setOutputValue(pinHandle, Board_LED2,!PIN_getOutputValue(Board_LED2));
    }
    else
    {
        /* Toggle LED1 and LED2 to indicate error */
        PIN_setOutputValue(pinHandle, Board_LED1,!PIN_getOutputValue(Board_LED1));
        PIN_setOutputValue(pinHandle, Board_LED2,!PIN_getOutputValue(Board_LED2));
    }

    Semaphore_post(txDoneSem);
}
#endif //RFEASYLINKTX_ASYNC

static void rfEasyLinkTxFnx(UArg arg0, UArg arg1)
{
	/* Create a semaphore for Async */
	Semaphore_Params params;
	Error_Block eb;

	/* Init params */
	Semaphore_Params_init(&params);
	Error_init(&eb);

	/* Create semaphore instance */
	txDoneSem = Semaphore_create(0, &params, &eb);
	txSwiSem = Semaphore_create(0, &params, &eb);

	Clock_start(clockRfTxn);

	while(1) {

		Semaphore_pend(txSwiSem, (BIOS_WAIT_FOREVER));

		EasyLink_TxPacket txPacket =  { {0}, 0, 0, {0} };

		txPacket.payload[0] = 0x39;
		txPacket.len = 1;
		txPacket.dstAddr[0] = 0xab;
		txPacket.absTime = 0;

		EasyLink_abort();
		EasyLink_transmitAsync(&txPacket, txDoneCb);

		Semaphore_pend(txDoneSem, (BIOS_WAIT_FOREVER));

		/* Tx Done.. Back to receive */
		EasyLink_receiveAsync(rxDoneCb, 0);
	}
}

void txTask_init(PIN_Handle inPinHandle) {
    pinHandle = inPinHandle;

    Task_Params_init(&txTaskParams);
    txTaskParams.stackSize = RFEASYLINKTX_TASK_STACK_SIZE;
    txTaskParams.priority = RFEASYLINKTX_TASK_PRIORITY;
    txTaskParams.stack = &txTaskStack;
    txTaskParams.arg0 = (UInt)1000000;

    Task_construct(&txTask, rfEasyLinkTxFnx, &txTaskParams, NULL);
}

static void rfEasyLinkRxFnx(UArg arg0, UArg arg1)
{
#ifndef RFEASYLINKRX_ASYNC
    EasyLink_RxPacket rxPacket = {0};
#endif

#ifdef RFEASYLINKRX_ASYNC
    /* Create a semaphore for Async*/
    Semaphore_Params params;
    Error_Block eb;

    /* Init params */
    Semaphore_Params_init(&params);
    Error_init(&eb);

    char uartTxBuffer[20];

    /* Create semaphore instance */
    rxDoneSem = Semaphore_create(0, &params, &eb);
#endif //RFEASYLINKRX_ASYNC

    EasyLink_init(EasyLink_Phy_50kbps2gfsk);
    EasyLink_setFrequency(868000000);
	EasyLink_setRfPwr(12);


#ifdef RFEASYLINKRX_ADDR_FILTER
    uint8_t addrFilter = 0xaa;
    EasyLink_enableRxAddrFilter(&addrFilter, 1, 1);
#endif //RFEASYLINKRX_ADDR_FILTER

    /* Initialize display and try to open both UART display. */
    Display_Params dispParams;
    Display_Params_init(&dispParams);
    dispParams.lineClearMode = DISPLAY_CLEAR_BOTH;

    /* Open both an available LCD display and an UART display.
     * Whether the open call is successful depends on what is present in the
     * Display_config[] array of the board file.
     *
     * Note that for SensorTag evaluation boards combined with the SHARP96x96
     * Watch DevPack, there is a pin conflict with UART such that one must be
     * excluded, and UART is preferred by default. To display on the Watch
     * DevPack, add the precompiler define BOARD_DISPLAY_EXCLUDE_UART.
     */
    hDisplaySerial = Display_open(Display_Type_UART, &dispParams);

    /* Check if the selected Display type was found and successfully opened */
    if (hDisplaySerial)
    {
        Display_print0(hDisplaySerial, 0, 0, "Concentrator...");
    }

    txTask_init(ledPinHandle);

    while(1) {
        EasyLink_receiveAsync(rxDoneCb, 0);
        Semaphore_pend(rxDoneSem, (BIOS_WAIT_FOREVER));

        /* print results */
        Display_print0(hDisplaySerial, 0, 0, "Conversion results:");
//        uint8_t i;
		/*for(i = 0; i !=8; i++)
		{
			System_sprintf(uartTxBuffer,"%u", result_buf[i]);
			Display_print0(hDisplaySerial, 0, 0, uartTxBuffer);
		}*/
        System_sprintf(uartTxBuffer,"%d",(int16_t)node_indentifier);
        Display_print0(hDisplaySerial, 0, 0, uartTxBuffer);
    }
}

void rxTask_init(PIN_Handle ledPinHandle) {
    pinHandle = ledPinHandle;

    Task_Params_init(&rxTaskParams);
    rxTaskParams.stackSize = RFEASYLINKEX_TASK_STACK_SIZE;
    rxTaskParams.priority = RFEASYLINKEX_TASK_PRIORITY;
    rxTaskParams.stack = &rxTaskStack;
    rxTaskParams.arg0 = (UInt)1000000;

    Task_construct(&rxTask, rfEasyLinkRxFnx, &rxTaskParams, NULL);
}

// 1s
void radioTxFn()
{
//	PIN_setOutputValue(pinHandle, Board_LED1,!PIN_getOutputValue(Board_LED1));
	Semaphore_post(txSwiSem);
}

/*
 *  ======== main ========
 */
int main(void)
{
    /* Call board init functions. */
    Board_initGeneral();

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if(!ledPinHandle) {
        System_abort("Error initializing board LED pins\n");
    }

    /* Clear LED pins */
    PIN_setOutputValue(ledPinHandle, Board_LED1, 0);
    PIN_setOutputValue(ledPinHandle, Board_LED2, 0);

    /* Initialise the UART and SPI for the display driver. */
    Board_initUART();

//    txTask_init(ledPinHandle);
    rxTask_init(ledPinHandle);

    /* Start BIOS */
    BIOS_start();

    return (0);
}
