/******************************************************************************
* File Name: main.c
*
* Description: This is the source code for the Bluetooth LE Throughput Client example for ModusToolbox.
*
* Related Document: See README.md
*
********************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
*        Header Files
*******************************************************************************/

#include <string.h>
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "GeneratedSource/cycfg_bt_settings.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_stack.h"
#include "cyabs_rtos.h"
#include "ble_client.h"

#ifdef ENABLE_BT_SPY_LOG
#include "cybt_debug_uart.h"
#endif

/*******************************************************************************
*        Macros
*******************************************************************************/

#define WRITE_TASK_NAME        "Write Task"
#define TPUT_TASK_NAME         "TPUT task"
#define BUTTON_TASK_NAME       "Button task"
#define TASK_STACK_SIZE         (3000)
#define TASK_PRIORITY        (CY_RTOS_PRIORITY_NORMAL)

/**
 * Task Handles for write_task, throughput_task and button_task
 */
cy_thread_t write_task_pointer, tput_task_pointer, button_task_pointer;
cy_semaphore_t congestion, interrupt, semaphore;

static uint64_t write_task_stack[TASK_STACK_SIZE/8];
static uint64_t tput_task_stack[TASK_STACK_SIZE/8];
static uint64_t button_task_stack[TASK_STACK_SIZE/8];

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization. The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
int main()
{
    cy_rslt_t rslt;
    wiced_result_t result;

    /* Initialize the board support package */
    rslt = cybsp_init();
    if (CY_RSLT_SUCCESS != rslt)
    {
        printf("BSP init failed \n");
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

#ifdef ENABLE_BT_SPY_LOG
{
    cybt_debug_uart_config_t config = {
        .uart_tx_pin = CYBSP_DEBUG_UART_TX,
        .uart_rx_pin = CYBSP_DEBUG_UART_RX,
        .uart_cts_pin = CYBSP_DEBUG_UART_CTS,
        .uart_rts_pin = CYBSP_DEBUG_UART_RTS,
        .baud_rate = DEBUG_UART_BAUDRATE,
        .flow_control = TRUE};
    cybt_debug_uart_init(&config, NULL);
}
#else
{
    /* Initialize retarget-io to use the debug UART port */
    rslt = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
}
#endif //ENABLE_BT_SPY_LOG

    printf("**** BLE Throughput Measurement - Client Application ****\n\n");

    /* Register call back and configuration with stack */
    result = wiced_bt_stack_init(app_bt_management_callback,
                                 &wiced_bt_cfg_settings);

    /* Check if stack initialization was successful */
    if( WICED_BT_SUCCESS == result)
    {
        printf("Bluetooth Stack Initialization Successful \n");
    }
    else
    {
        printf("Bluetooth Stack Initialization failed!! \n");
        CY_ASSERT(0);
    }

    /*Create Write task*/
     result = cy_rtos_thread_create(&write_task_pointer,
                                    &write_task,
                                    WRITE_TASK_NAME,
                                    &write_task_stack,
                                    TASK_STACK_SIZE,
                                    TASK_PRIORITY,
                                    0);
     if (result != CY_RSLT_SUCCESS)
     {
         printf("Write task creation failed 0x%X\n", result);
     }
     else
     {
         printf("Write task created successfully\n");
     }

     /* Create Throughput task*/
     result = cy_rtos_thread_create(&tput_task_pointer,
                                    &tput_task,
                                    TPUT_TASK_NAME,
                                    &tput_task_stack,
                                    TASK_STACK_SIZE,
                                    TASK_PRIORITY,
                                    0);
     if (result != CY_RSLT_SUCCESS)
     {
         printf("Throughput task creation failed 0x%X\n", result);
     }
     else
     {
         printf("Throughput task created successfully\n");
     }

     /* Create Button task */
     result = cy_rtos_thread_create(&button_task_pointer,
                                   &button_task,
                                   BUTTON_TASK_NAME,
                                   &button_task_stack,
                                   TASK_STACK_SIZE,
                                   TASK_PRIORITY,
                                   0);
     if (result != CY_RSLT_SUCCESS)
     {
         printf("Button task creation failed 0x%X\n", result);
     }
     else
     {
         printf("Button task created successfully\n");
     }

     /* Initialize the semaphores */
     result = cy_rtos_semaphore_init(&congestion, 1, 0);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Congestion semaphore initialization failed 0x%X\n", result);
    }

    result = cy_rtos_semaphore_init(&interrupt, 1, 0);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Button interrupt semaphore initialization failed 0x%X\n", result);
    }

    result = cy_rtos_semaphore_init(&semaphore, 1, 0);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Semaphore initialization failed 0x%X\n", result);
    }
}


/* END OF FILE */
