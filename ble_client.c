/*******************************************************************************
 * File Name: ble_client.c
 *
 * Description: This is the source code for the Threadx: BLE Throughput Client
 *              Example for ModusToolbox.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
 ******************************************************************************/

/*******************************************************************************
*        Header Files
*******************************************************************************/

#include "string.h"
#include "cyhal.h"
#include "cy_retarget_io.h"
#include "app_bt_utils.h"
#include "wiced_memory.h"
#include "wiced_bt_stack.h"
#include "cycfg_gap.h"
#include "cyhal_gpio.h"
#include "wiced_bt_l2c.h"
#include "cyabs_rtos.h"
#include "ble_client.h"
#include "app_bt_utils.h"

#ifdef ENABLE_BT_SPY_LOG
#include "cybt_debug_uart.h"
#endif

/*******************************************************************************
*         Macros
*******************************************************************************/

#define WRITE_DATA_SIZE                       (244)
#define TPUT_TIMER_UPDATE                   (5*3000000)
#define TPUT_FREQUENCY                       (3000000)
#define TIMER_INTERRUPT_PRIORITY				(3)
#define PACKET_PER_EVENT						(10)

/*******************************************************************************
*        Variable Definitions
*******************************************************************************/

/* Variables to hold GATT notification bytes sent and GATT Write bytes received
 * successfully
 */
static unsigned long gatt_notif_rx_bytes = 0;
static unsigned long gatt_write_tx_bytes = 0;

/* Variable that stores the data which will be sent as GATT write */
uint8_t write_data_seq[WRITE_DATA_SIZE];

/* Variable to store Scan and connection state*/
static app_bt_scan_conn_mode_t app_bt_scan_conn_state = APP_BT_SCAN_OFF_CONN_OFF;

/* Variable to store the status of service found or not on the Server */
static bool tput_service_found = false;

/* Variable to store connection state information*/
static conn_state_info_t conn_state_info;

/* Enable or Disable notification from server */
static bool enable_cccd = true;

/* Flag to enable or disable GATT write */
static bool gatt_write_tx = false;

/* Flag used to Scan only for first button press */
static bool scan_flag = true;

/* Variable to switch between different data transfer modes */
static tput_mode_t mode_flag = GATT_NOTIFANDWRITE;

/* Array to store the service UUID to be search in the Server GATT database */
static const uint8_t tput_service_uuid[LEN_UUID_128] = TPUT_SERVICE_UUID;

/* Variable to store the handle of the service found */
static uint16_t tput_service_handle = 0;

uint8_t tput_fun = 1;

static wiced_bt_gatt_write_hdr_t tput_write_cmd = {0};

/* Variables for app buffer + handling */
uint8_t *tput_buffer_ptr;
uint8_t  value_initialize = 243;

/**
 * @brief Variable for 5 sec timer object
 */
static cyhal_timer_t tput_timer_obj;

/**
 * @brief Configure timer for 5 sec
 */
const cyhal_timer_cfg_t tput_timer_cfg =
    {
        .compare_value = 0,                    /* Timer compare value, not used */
        .period = TPUT_TIMER_UPDATE,           /* Defines the timer period */
        .direction = CYHAL_TIMER_DIR_UP,       /* Timer counts up */
        .is_compare = false,                   /* Don't use compare mode */
        .is_continuous = true,                 /* Run timer indefinitely */
        .value = 0                             /* Initial value of counter */
};

/*******************************************************************************
*        Function Prototypes
*******************************************************************************/

static void tput_ble_app_init(void);
static wiced_bt_gatt_status_t tput_enable_disable_gatt_notification(bool notify);
static void tput_scan_result_cback(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data);

static void button_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);

/* GATT Event Callback Functions */
static wiced_bt_gatt_status_t ble_app_connect_callback(wiced_bt_gatt_connection_status_t *p_conn_status);
static wiced_bt_gatt_status_t ble_app_gatt_event_handler(wiced_bt_gatt_evt_t event,
                                                         wiced_bt_gatt_event_data_t *p_event_data);

/* HAL timer callback registered when timer reaches terminal count */
void tput_timer_callb(void *callback_arg, cyhal_timer_event_t event);


static cyhal_gpio_callback_data_t cyhal_gpio_callback_data =
{
        .callback = button_interrupt_handler,
        .pin = CYBSP_USER_BTN,
};

/******************************************************************************
 * Function Definitions
 ******************************************************************************/

 /**
 * Function Name: hci_trace_cback
 *
 * Function Description:
 *   @brief This callback routes HCI packets to debug uart.
 *
 *   @param wiced_bt_hci_trace_type_t type : HCI trace type
 *   @param uint16_t length : length of p_data
 *   @param uint8_t* p_data : pointer to data
 *
 *   @return None
 *
 */
#ifdef ENABLE_BT_SPY_LOG
void hci_trace_cback(wiced_bt_hci_trace_type_t type,
                     uint16_t length, uint8_t* p_data)
{
    cybt_debug_uart_send_hci_trace(type, length, p_data);
}
#endif

/**
* Function Name: app_bt_alloc_buffer()
*
* @Brief  This function allocates the memory buffer
*
* @Params   uint16_t len : Length to allocate
*
* @Return    uint8_t* : pointer to the allocated memory buffer
*
*/
static uint8_t *app_bt_alloc_buffer(uint16_t len)
{
    uint8_t *p = (uint8_t *)malloc(len);
    printf( "%s() len %d alloc %p \r\n", __FUNCTION__,len, p);
    return p;
}

/**
* Function Name: app_bt_alloc_buffer()
*
* @Brief  This function frees up the memory buffer
*
* @Params   uint8_t*  : pointer to the buffer to be freed
*
* @Return    void
*
*/
static void app_bt_free_buffer(uint8_t *p_data)
{
    if (p_data != NULL)
    {
        printf( "%s()        free:%p \r\n",__FUNCTION__, p_data);
        free(p_data);
    }
}

/**
* Function Name: app_bt_management_callback()
*
* @Brief  This is a Bluetooth stack event handler function to receive management events
*         from the BLE stack and process as per the application.
*
* @Params   wiced_bt_management_evt_t      : BLE event code of one byte length
*           wiced_bt_management_evt_data_t : Pointer to BLE management event structures
*
* @Return    wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
*/
wiced_result_t app_bt_management_callback(wiced_bt_management_evt_t event,
                                          wiced_bt_management_evt_data_t *p_event_data)
{

    wiced_result_t status = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda = {0};
    wiced_bt_ble_scan_type_t p_scan_type ;

    switch (event)
    {
    case BTM_ENABLED_EVT:                  /* Bluetooth Controller and Host Stack Enabled */
        if(WICED_BT_SUCCESS == p_event_data->enabled.status)
        {
            wiced_bt_dev_read_local_addr(bda);
            printf("Local Bluetooth Address: ");
            print_bd_address(bda);
            /* Perform application-specific initialization */
            tput_ble_app_init();
        }
        else
        {
            printf("Bluetooth Disabled \n");
        }
        break;

    case BTM_BLE_SCAN_STATE_CHANGED_EVT:               /* Scan State Changed */
        p_scan_type = p_event_data->ble_scan_state_changed;

        if(BTM_BLE_SCAN_TYPE_NONE == p_scan_type)
        {
            printf("Scanning stopped\n");
            /* Check connection status after scanning stops and
             * update the scan+conn state variable */
            if (conn_state_info.conn_id == 0)
            {
                app_bt_scan_conn_state = APP_BT_SCAN_OFF_CONN_OFF;
            }
            else
            {
                app_bt_scan_conn_state = APP_BT_SCAN_OFF_CONN_ON;
            }
        }
        else
        {
            printf("Scanning.....\n");
            app_bt_scan_conn_state = APP_BT_SCAN_ON_CONN_OFF;
        }

        break;

    case BTM_BLE_PHY_UPDATE_EVT:                /* Physical link (PHY) updated */
        conn_state_info.rx_phy = p_event_data->ble_phy_update_event.rx_phy;
        conn_state_info.tx_phy = p_event_data->ble_phy_update_event.tx_phy;
        printf("Selected RX PHY - %dM\nSelected TX PHY - %dM\n", conn_state_info.rx_phy,conn_state_info.tx_phy);
        break;

    case BTM_BLE_CONNECTION_PARAM_UPDATE:            /* Connection parameters updated */

        if(WICED_BT_SUCCESS == p_event_data->ble_connection_param_update.status)
        {
            conn_state_info.conn_interval = (double)((p_event_data->ble_connection_param_update.conn_interval));
            printf("New connection interval: %f ms\n", conn_state_info.conn_interval);
        }
        else
        {
            printf("Connection parameters update failed: %d\n", p_event_data->ble_connection_param_update.status);
        }
        break;

    default:
        printf("Unhandled Bluetooth Management Event: 0x%x %s\n", event, get_bt_event_name(event));
        break;
    }

    return status;
}

/**
* Function Name: tput_ble_app_init()
*
* @Brief  This function handles application level initialization tasks and is called
*         from the BT management callback once the BLE stack enabled event
*         (BTM_ENABLED_EVT) is triggered. This function is executed in the
*         BTM_ENABLED_EVT management callback.
*
* @Params  None
*
*@Return   None
*
*/
static void tput_ble_app_init(void)
{
    cy_rslt_t rslt = CY_RSLT_SUCCESS;
    cy_rslt_t cy_result = CY_RSLT_SUCCESS;
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

#ifdef ENABLE_BT_SPY_LOG
    wiced_bt_dev_register_hci_trace(hci_trace_cback);
#endif

    /*Initialize the data packets to be sent as GATT write data to the peer device */
    for(uint8_t index = 0; index < WRITE_DATA_SIZE; index++)
    {
        write_data_seq[index] = index;
    }

    /* Initialize GPIO for button interrupt*/
    rslt = cyhal_gpio_init(CYBSP_USER_BTN,
                           CYHAL_GPIO_DIR_INPUT,
                           CYHAL_GPIO_DRIVE_PULLUP,
                           CYBSP_BTN_OFF);
    /* GPIO init failed. Stop program execution */
    if (CY_RSLT_SUCCESS != rslt)
    {
        printf("Button GPIO initialisation failed! \n");
        CY_ASSERT(0);
    }

    /* Configure GPIO interrupt */
    cyhal_gpio_register_callback(CYBSP_USER_BTN,
                                &cyhal_gpio_callback_data);
    cyhal_gpio_enable_event(CYBSP_USER_BTN,
                            CYHAL_GPIO_IRQ_FALL,
                            GPIO_INTERRUPT_PRIORITY,
                            true);

    /* Initialize the HAL timer used to count seconds */
    cy_result = cyhal_timer_init(&tput_timer_obj, NC, NULL);
    if (CY_RSLT_SUCCESS != cy_result)
    {
        printf("Throughput timer init failed !\n");
    }
    /* Configure the timer for 5 seconds */
    cyhal_timer_configure(&tput_timer_obj, &tput_timer_cfg);
    cy_result = cyhal_timer_set_frequency(&tput_timer_obj, TPUT_FREQUENCY);
    if (CY_RSLT_SUCCESS != cy_result)
    {
        printf("Throughput timer set freq failed !\n");
    }

    /* Register for a callback whenever timer reaches terminal count */
    cyhal_timer_register_callback(&tput_timer_obj, tput_timer_callb, NULL);
    cyhal_timer_enable_event(&tput_timer_obj, CYHAL_TIMER_IRQ_TERMINAL_COUNT, TIMER_INTERRUPT_PRIORITY, true);

    /* Disable pairing for this application */
    wiced_bt_set_pairable_mode(WICED_FALSE, WICED_FALSE);

    /* Register with BT stack to receive GATT callback */
    status = wiced_bt_gatt_register(ble_app_gatt_event_handler);
    printf("GATT event Handler registration status: %s \n", get_bt_gatt_status_name(status));

    if (WICED_BT_GATT_SUCCESS != status)
    {
        printf("GATT Registeration failed  because of error: %d \n", status);
        CY_ASSERT(0);
    }

    /* Initialize GATT Database */
    status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len, NULL);
    printf("GATT database initialization status: %s \n", get_bt_gatt_status_name(status));

    if (WICED_BT_GATT_SUCCESS != status)
    {
        printf("GATT initialization failed because of error: %d \n", status);
        CY_ASSERT(0);
    }

    printf("Press User Button on your kit to start scanning.....\n");

    /* Start tput timer */
    if (CY_RSLT_SUCCESS != cyhal_timer_start(&tput_timer_obj))
    {
       printf("Throughput timer start failed !");
       CY_ASSERT(0);
    }
}

/**
* Function Name: ble_app_gatt_event_handler()
*
* @Brief    This function handles GATT events from the BT stack.
*
* @Params   wiced_bt_gatt_evt_t event : BLE GATT event code of one byte length
*           wiced_bt_gatt_event_data_t *p_event_data : Pointer to BLE GATT event structures
*
* @Return   wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
*                                   in wiced_bt_gatt.h
*
*/
static wiced_bt_gatt_status_t ble_app_gatt_event_handler(wiced_bt_gatt_evt_t event,
                                                         wiced_bt_gatt_event_data_t *p_event_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    /* Call the appropriate callback function based on the GATT event type, and
     * pass the relevant event parameters to the callback function */

    switch (event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        status = ble_app_connect_callback(&p_event_data->connection_status);
        break;

    case GATT_DISCOVERY_RESULT_EVT:
        /* Check if it is throughput service uuid */
        if (!memcmp(&p_event_data->discovery_result.discovery_data.group_value.service_type.uu.uuid128,
                    &tput_service_uuid, TPUT_SERVICE_UUID_LEN))
        {
            /* Update the handle to TPUT service uuid,
             * Throughput service GATT handle : 0x0009 */
            tput_service_handle = p_event_data->discovery_result.discovery_data.group_value.s_handle;
            tput_service_found = true;
        }
        break;

    case GATT_DISCOVERY_CPLT_EVT:
        if (tput_service_found)
        {
            printf("Custom throughput service found\n");
        }
        else
        {
            printf("Custom throughput service not found\n");
        }
        break;

    case GATT_OPERATION_CPLT_EVT:
        switch (p_event_data->operation_complete.op)
        {
        case GATTC_OPTYPE_WRITE_WITH_RSP:
        /*Check if GATT operation of enable/disable notification is success.*/
            if ((p_event_data->operation_complete.response_data.handle ==
                               (tput_service_handle + GATT_CCCD_HANDLE)) &&
                               (WICED_BT_GATT_SUCCESS == p_event_data->operation_complete.status))
            {
                printf("Notifications %s\n",(enable_cccd)?"enabled":"disabled");
                /* Start write timer only for GATT writes */
                if (gatt_write_tx)
                {
                    /* Clear GATT Tx packets */
                    gatt_notif_rx_bytes = 0;
                }
            }
            else if (p_event_data->operation_complete.response_data.handle ==
                                   (tput_service_handle + GATT_CCCD_HANDLE) &&
                                   (WICED_BT_GATT_SUCCESS != p_event_data->operation_complete.status))
            {
                printf("CCCD update failed. Error: %x\n", p_event_data->operation_complete.status);
            }
            break;

        case GATTC_OPTYPE_WRITE_NO_RSP:
            if ((p_event_data->operation_complete.response_data.handle ==
                               (tput_service_handle + GATT_WRITE_HANDLE)) &&
                               (WICED_BT_GATT_SUCCESS == p_event_data->operation_complete.status))
            {
                gatt_write_tx_bytes += WRITE_DATA_SIZE;
            }
            break;

        case GATTC_OPTYPE_NOTIFICATION:
            /* Receive GATT Notifications from server */
            gatt_notif_rx_bytes += p_event_data->operation_complete.response_data.att_value.len;

            break;

        case GATTC_OPTYPE_CONFIG_MTU:
            conn_state_info.mtu = p_event_data->operation_complete.response_data.mtu;
            printf("Negotiated MTU Size: %d\n", conn_state_info.mtu);

            /* Send GATT service discovery request */
            wiced_bt_gatt_discovery_param_t gatt_discovery_setup = {0};
            gatt_discovery_setup.s_handle = 1;
            gatt_discovery_setup.e_handle = 0xFFFF;
            gatt_discovery_setup.uuid.len = TPUT_SERVICE_UUID_LEN;
            memcpy(gatt_discovery_setup.uuid.uu.uuid128,
                   tput_service_uuid,
                   TPUT_SERVICE_UUID_LEN);

            status = wiced_bt_gatt_client_send_discover(conn_state_info.conn_id,
                                                        GATT_DISCOVER_SERVICES_BY_UUID,
                                                        &gatt_discovery_setup);
            if (WICED_BT_GATT_SUCCESS != status)
            {
                printf("GATT Discovery request failed. Error code: %d,Conn id: %d\n",
                                               status, conn_state_info.conn_id);
            }
            break;
        }
        break;

    case GATT_CONGESTION_EVT:
        if(!p_event_data->congestion.congested)
        {
            cy_rtos_semaphore_set(&congestion);
        }
        break;

    case GATT_GET_RESPONSE_BUFFER_EVT:
        if (p_event_data->buffer_request.len_requested != 0)
        {
            p_event_data->buffer_request.buffer.p_app_rsp_buffer =
                          app_bt_alloc_buffer(p_event_data->buffer_request.len_requested);
            p_event_data->buffer_request.buffer.p_app_ctxt = (void *)app_bt_free_buffer;
            status = WICED_BT_GATT_SUCCESS;
        }
        break;
    case GATT_APP_BUFFER_TRANSMITTED_EVT:
    {
        break;
    }

    default:
        status = WICED_BT_GATT_SUCCESS;
        break;
    }
    return status;
}

/**
 * Function Name: tput_button_interrupt_handler
 *
 * @Brief     GPIO interrupt service routine. This function detects button presses and
 *            invokes the button task
 *
 * @Params    void *handler_arg : pointer to the variable passed to the ISR
 *            cyhal_gpio_event_t event : GPIO event type
 *
 * @Return    void
 *
 */
static void button_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
    cy_rtos_semaphore_set(&interrupt);
}

/**
 * Function Name: button_task
 *
 * @Brief   This function handles the button event.
 *
 * @Params  cy_thread_arg_t : Unused
 *
 * @Return  void
 *
 */
void button_task(cy_thread_arg_t arg)
{
    wiced_result_t status = WICED_BT_SUCCESS;
    wiced_bt_gatt_status_t gatt_status;
    while (true)
    {
        cy_rtos_semaphore_get(&interrupt, CY_RTOS_NEVER_TIMEOUT);
        if (!conn_state_info.conn_id)
        {
            if (scan_flag)
            {
                /* Start scan */
                status = wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_HIGH_DUTY, true,
                                           tput_scan_result_cback);
                if ((WICED_BT_PENDING != status) && (WICED_BT_BUSY != status))
                {
                    printf("Error: Starting scan failed. Error code: %d\n",status);
                    /* Switch off the scan LED */
                    app_bt_scan_conn_state = APP_BT_SCAN_OFF_CONN_OFF;
                }
            }
        }
        else
        {
            /* After connection pressing the user button will change the
            * throughput modes as follows :
            * GATT_Notif_StoC -> GATT_Write_CtoS -> GATT_NotifandWrite -> Roll
            * back to GATT_Notif_StoC
            */
            gatt_write_tx_bytes = 0;

            /* Change data transfer modes upon interrupt. Based on the current
            * mode,set flags to enable/disable notifications and set/clear GATT
            * write flag
            */
            mode_flag = (mode_flag == GATT_NOTIFANDWRITE) ?
                         GATT_NOTIF_STOC : (tput_mode_t)(mode_flag + 1u);

            printf("mode flag %d\r\n",mode_flag);

            switch (mode_flag)
            {
            case GATT_NOTIF_STOC:
                enable_cccd = true;
                gatt_write_tx = false;
                break;

            case GATT_WRITE_CTOS:
                enable_cccd = false;
                gatt_write_tx = true;
                break;

            case GATT_NOTIFANDWRITE:
                enable_cccd = true;
                gatt_write_tx = true;
                break;

            default:
                printf("Invalid Data Transfer Mode\n");
                break;
            }

            printf("Enable ccdd %d\n",enable_cccd);

            /*Delay added to avoid the failure of notification enable packet*/
            cy_rtos_delay_milliseconds(2000);
            gatt_status = tput_enable_disable_gatt_notification(enable_cccd);

            if (WICED_BT_GATT_SUCCESS != gatt_status)
            {
                printf("Enable/Disable notification failed: %d\n\r",gatt_status);
            }
        }
    }
}


/**
* Function Name: tput_scan_result_cback()
*
* @Brief    This function is registered as a callback to handle the scan results.
*           When the desired device is found, it will try to establish connection with
*           that device.
*
* @Params   wiced_bt_ble_scan_results_t *p_scan_result : Details of the new device found.
*           uint8_t *p_adv_data : Advertisement data.
*
* @Return   void
*
*/
static void tput_scan_result_cback(wiced_bt_ble_scan_results_t *p_scan_result,
                                   uint8_t *p_adv_data)
{
    wiced_result_t status = WICED_BT_SUCCESS;
    uint8_t length = 0u;
    uint8_t *p_data = NULL;
    uint8_t server_device_name[5] = {'T', 'P', 'U', 'T', '\0'};

    if (p_scan_result)
    {
        p_data = wiced_bt_ble_check_advertising_data(p_adv_data,
                                                     BTM_BLE_ADVERT_TYPE_NAME_COMPLETE,
                                                     &length);

        if (p_data != NULL)
        {
            /* Check if the peer device's name is "TPUT" */
            if ((length = strlen((const char *)server_device_name)) &&
                (memcmp(p_data, (uint8_t *)server_device_name, length) == 0))
            {
                printf("Scan completed\n Found peer device with BDA:\n");
                print_bd_address(p_scan_result->remote_bd_addr);
                scan_flag = false;

                /* Device found. Stop scanning */
                if ((status = wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_NONE, true,
                                                tput_scan_result_cback)) != 0)
                {
                    printf("Scan off status %d\n", status);
                }

                /* Initiate the connection */
                if (wiced_bt_gatt_le_connect(p_scan_result->remote_bd_addr,
                                             p_scan_result->ble_addr_type,
                                             BLE_CONN_MODE_HIGH_DUTY,
                                             WICED_TRUE) != WICED_TRUE)
                {
                    printf("wiced_bt_gatt_connect failed\n");
                }
                else
                {
                    printf("gatt connect request sent\n");
                }
            }
        }
    }
}

/**
* Function Name: ble_app_connect_callback()
*
* @Brief    This callback function handles connection status changes.
*
* @Params   wiced_bt_gatt_connection_status_t *p_conn_status  : Pointer to data that has
*                                                               connection details
*
* @Return   wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
*                                    in wiced_bt_gatt.h
*
*/
static wiced_bt_gatt_status_t ble_app_connect_callback(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    if (NULL != p_conn_status)
    {
        if (p_conn_status->connected)
        {
            /* Device has connected */
            printf("Connected : BDA ");
            print_bd_address(p_conn_status->bd_addr);
            printf("Connection ID '%d'\n", p_conn_status->conn_id);

            /* Store the connection ID and remote BDA*/
            conn_state_info.conn_id = p_conn_status->conn_id;
            printf("connection_id = %d\n", conn_state_info.conn_id);
            memcpy(conn_state_info.remote_addr,
                        p_conn_status->bd_addr,
                        BD_ADDR_LEN);

            /* Update the scan/conn state */
            app_bt_scan_conn_state = APP_BT_SCAN_OFF_CONN_ON;

            wiced_bt_l2cap_enable_update_ble_conn_params(conn_state_info.remote_addr,
                                                         true);

            /* Send MTU exchange request */
            status = wiced_bt_gatt_client_configure_mtu(conn_state_info.conn_id,
                                                        CY_BT_RX_PDU_SIZE);
            if (status != WICED_BT_GATT_SUCCESS)
            {
                printf("GATT MTU configure failed %d\n", status);
            }
            else{
                printf("GATT MTU configured\n");
                printf("Packets %d\n",PACKET_PER_EVENT);
            }
        }
        else
        {
            /* Device has disconnected */
            printf("Disconnected : BDA ");
            print_bd_address(p_conn_status->bd_addr);
            printf("Connection ID '%d', Reason '%s'\n",
                        p_conn_status->conn_id,
                        get_bt_gatt_disconn_reason_name(p_conn_status->reason));

            /* Fill the structure containing connection info with zero */
            memset(&conn_state_info, 0, sizeof(conn_state_info));

            /* Reset the flags */
            tput_service_found = false;
            mode_flag = GATT_NOTIF_STOC;
            enable_cccd = true;
            gatt_write_tx = false;
            scan_flag = true;
            /* Clear tx and rx packet count */
            gatt_notif_rx_bytes = 0;
            gatt_write_tx_bytes = 0;

            /* Stop the tput timer */
            if (CY_RSLT_SUCCESS != cyhal_timer_stop(&tput_timer_obj))
            {
                printf("Throughput timer stop failed !\n");
                CY_ASSERT(0);
            }

            /* Update the scan/conn state */
            app_bt_scan_conn_state = APP_BT_SCAN_ON_CONN_OFF;
            printf("Press user button on your kit to start scanning.....\n");
        }
    }

    return status;
}


/*
 Function name:
 bas_timer_callb

 Function Description:
 @brief  This callback function is invoked on timeout of 1 second timer.

 @param  void*: unused
 @param cyhal_timer_event_t: unused

 @return void
 */
void tput_timer_callb(void *callback_arg, cyhal_timer_event_t event)
{
    cy_rslt_t result;
    tput_fun = 1;
    result = cy_rtos_thread_set_notification(&tput_task_pointer);
    if (result != CY_RSLT_SUCCESS)
    {
        if (result == CY_RTOS_GENERAL_ERROR)
        {
            printf("CY_RTOS_GENERAL_ERROR thread notification failed !\n");
        }
        else if (result == CY_RTOS_BAD_PARAM)
        {
            printf("CY_RTOS_BAD_PARAM thread notification failed !\n");
        }
        else
        {
            printf("others thread notification failed !\n");
        }
    }
}

/**
 * Function Name: tput_task
 *
 * @Brief   This function calculates the throughput values
 *
 * @Params  cy_thread_arg_t : Unused
 *
 * @Return  void
 *
 */
void tput_task(cy_thread_arg_t arg)
{
    while(true){
        cy_rtos_thread_wait_notification(CY_RTOS_NEVER_TIMEOUT);

        if (conn_state_info.conn_id && gatt_notif_rx_bytes)
        {
            gatt_notif_rx_bytes = (gatt_notif_rx_bytes * 8) / (5*1000);
            printf("GATT NOTIFICATION : Client Throughput (RX) = %lu kbps\n", gatt_notif_rx_bytes);
            gatt_notif_rx_bytes = 0; //Reset the byte counter
        }

        if ((conn_state_info.conn_id) && gatt_write_tx_bytes)
        {
            gatt_write_tx_bytes = (gatt_write_tx_bytes * 8) / (5*1000);
            printf("GATT WRITE        : Client Throughput (TX) = %lu kbps\n", gatt_write_tx_bytes);
            gatt_write_tx_bytes = 0; //Reset the byte counter
        }

        cy_rtos_semaphore_set(&semaphore);
        tput_fun = 0;
    }
}


/**
 * Function Name: write_task
 *
 * @Brief   This function sends gatt write with no response when GATT WRITE is enabled
 *
 * @Params  cy_thread_arg_t : Unused
 *
 * @Return  void
 *
 */
void write_task(cy_thread_arg_t arg)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    while(true)
    {
        /* Send GATT write(with no response) commands to the server only
         * when there is no GATT congestion and no GATT notifications are being
         * received. In data transfer mode 3(Both TX and RX), the GATT write
         * commands will be sent irrespective of GATT notifications being received
         * or not.
         */

        if(tput_fun == 1){
            cy_rtos_semaphore_get(&semaphore, CY_RTOS_NEVER_TIMEOUT);
        }

        if (gatt_write_tx == true)
        {
            tput_write_cmd.auth_req = GATT_AUTH_REQ_NONE;
            tput_write_cmd.handle = (tput_service_handle) + GATT_WRITE_HANDLE;
            tput_write_cmd.len = WRITE_DATA_SIZE;
            tput_write_cmd.offset = 0;

            for(int i=0; i<PACKET_PER_EVENT; i++){
                status = wiced_bt_gatt_client_send_write(conn_state_info.conn_id,
                                                                    GATT_CMD_WRITE,
                                                                    &tput_write_cmd,
                                                                    write_data_seq,
                                                                    (void *)app_bt_free_buffer);

                if(WICED_BT_GATT_CONGESTED == status)
                {
                	printf("GATT Congestion\r\n");
                    app_bt_free_buffer((wiced_bt_buffer_t *)tput_buffer_ptr);
                    cy_rtos_semaphore_get(&congestion, CY_RTOS_NEVER_TIMEOUT);
                }
            }
            cy_rtos_delay_milliseconds(10);
        }
        else{
            cy_rtos_delay_milliseconds(1000);
        }
    }
}


/**
* Function Name: tput_enable_disable_gatt_notification()
*
* @Brief        Enable or disable  GATT notification from the server.
*
* @Params       bool notify : Boolean variable to enable/disable notification.
*
* @Return        wiced_bt_gatt_status_t  : Status code from wiced_bt_gatt_status_e.
*
*/
static wiced_bt_gatt_status_t tput_enable_disable_gatt_notification(bool notify)
{
    wiced_bt_gatt_write_hdr_t tput_write_notif = {0};
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
    uint8_t local_notif_enable[CCCD_LENGTH] = {0};
    uint8_t *notif_val = NULL;

    /*Allocate memory for data to be written on server DB and pass it to stack*/
    notif_val = app_bt_alloc_buffer(sizeof(uint16_t)); //CCCD is two bytes

    if (notif_val)
    {
        local_notif_enable[0] = notify;
        memcpy(notif_val, local_notif_enable, sizeof(uint16_t));
        tput_write_notif.auth_req = GATT_AUTH_REQ_NONE;
        tput_write_notif.handle = tput_service_handle + GATT_CCCD_HANDLE;
        tput_write_notif.len = CCCD_LENGTH;
        tput_write_notif.offset = 0;
        status = wiced_bt_gatt_client_send_write(conn_state_info.conn_id,
                                                GATT_REQ_WRITE,
                                                &tput_write_notif,
                                        notif_val,(void *)app_bt_free_buffer);
    }
    else
    {
        printf("malloc failed! write request not sent\n");
        status = WICED_BT_GATT_ERROR;
    }
    return status;
}


/* [] END OF FILE */
