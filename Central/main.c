/*
 * Copyright (c) Roanoke20 2015. All Rights Reserved.
 * [NOTE] Consider that application will restart if device disconnect, but timer is still going. It is caused by timer handler.
 */

		
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_sdm.h"
#include "ble.h"
#include "softdevice_handler.h"
#include "app_util.h"
#include "app_error.h"
#include "boards.h"
#include "nrf_gpio.h"
#include "pstorage.h"
#include "device_manager.h"
#include "app_trace.h"
#include "app_util.h"
#include "bsp.h"
#include "ble_db_discovery.h"
#include "lls_client.h"
#include "app_timer.h"
#include "ble_advdata.h"
#include "nrf_delay.h"

#ifdef BSP_BUTTON_1
#define BOND_DELETE_ALL_BUTTON_PIN BSP_BUTTON_1 /**< Button used for deleting all bonded centrals during startup. */
#endif


#ifdef JAVA_APP_IN_USE
		#undef ENABLE_DEBUG_LOG_SUPPORT 
#endif
	
#define APPL_LOG                   app_trace_log                     /**< Debug logger macro that will be used in this file to do logging of debug information over UART. */

#define SEC_PARAM_BOND             1//was 1. look at this!                                 /**< Perform bonding. */
#define SEC_PARAM_MITM             1 //was 0.                                 /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES  BLE_GAP_IO_CAPS_NONE              /**< No I/O capabilities. */
#define SEC_PARAM_OOB              1                                 /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE     7                                 /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE     16                                /**< Maximum encryption key size. */

#define SCAN_INTERVAL              0x00A0                            /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                0x0050                            /**< Determines scan window in units of 0.625 millisecond. */

#define MIN_CONN_INTERVAL          MSEC_TO_UNITS(500, UNIT_1_25_MS)  /**< Minimum acceptable connection interval (0.5 seconds).  */
#define MAX_CONN_INTERVAL          MSEC_TO_UNITS(1000, UNIT_1_25_MS) /**< Maximum acceptable connection interval (1 second). */

#define SLAVE_LATENCY              0                                 /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT        MSEC_TO_UNITS(4000, UNIT_10_MS)   /**< Determines supervision time-out in units of 10 millisecond. */

#define MAX_PEER_COUNT             DEVICE_MANAGER_MAX_CONNECTIONS /**< Maximum number of peer's application intends to manage. */

/* TIMER. 
 * It is used only in demonstarate version to show how peripheral reacts if the central is disconnected.
 * After pairing central writes high alert to lls characteristic on the peripheral. Next the timer is starting.
 * After 20 sec. central writes low alert to lls char. 
 */
#define APP_TIMER_PRESCALER        0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS       MAX_PEER_COUNT                              /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE    4                                           /**< Size of timer operation queues. */
#define LLS_INTERVAL               APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER) /**< Timer interval. It is equal 20 sec. */

/*< Key used to pairing. */
#define OOB_AUTH_KEY                 {                            \
                                            {                         \
                                              0xAA, 0xBB, 0xCC, 0xDD, \
                                              0xEE, 0xFF, 0x99, 0x88, \
                                              0x77, 0x66, 0x55, 0x44, \
                                              0x33, 0x22, 0x11, 0x00  \
                                            }                         \
                                       }

static dm_application_instance_t m_dm_app_id;                         /**< Application identifier. */
static uint8_t                   m_peer_count = 0;                    /**< Number of peer's connected. */
static ble_db_discovery_t        m_ble_db_discovery;                  /**< Structure used to identify the DB Discovery module. */
static bool                      m_memory_access_in_progress = false; /**< Flag to keep track of ongoing operations on persistent memory. */

//TIMER data
static app_timer_id_t            m_lls_timer[MAX_PEER_COUNT];					/**< App timers. */

//LLS Data
//static ble_lls_c_t   m_lls_client[MAX_PEER_COUNT];									/**< Table contains structures used to identify lls_clients. */
static ble_lls_c_t * p_lls_clients[MAX_PEER_COUNT];										/**< Pointers to ble_lls_c_t structures. */

//Data used to pairing.
static ble_gap_addr_t          * p_whitelist_addr[MAX_PEER_COUNT];		/**< List contains peripherals addreses. */
static ble_advdata_tk_value_t  * p_whitelist_tk[MAX_PEER_COUNT];			/**< List contains peripherals tk values. */
static uint8_t                   m_whitelist_actual_client_nbr = 0;   /**< Contains  nbr of actual client's in database. */
static ble_gap_scan_params_t		 m_scan_param; 												/**< Scan parameters requested for scanning and connection. */

static ble_gap_addr_t m_peer_addr_actual;
static uint8_t m_timer_nbr;
/**
 * @brief Connection parameters requested for connection.
 */
static const ble_gap_conn_params_t m_connection_param =
{
    (uint16_t)MIN_CONN_INTERVAL,  // Minimum connection
    (uint16_t)MAX_CONN_INTERVAL,  // Maximum connection
    0,                            // Slave latency
    (uint16_t)SUPERVISION_TIMEOUT // Supervision time-out
};

static void scan_start(void);
static uint32_t client_database_remove_client(ble_gap_addr_t * p_client_addr);
static uint8_t  client_get(ble_gap_addr_t * p_client_addr, uint8_t * p_client_nbr);
static uint32_t client_database_add_client(ble_gap_addr_t          * p_client_addr,
                                           ble_advdata_tk_value_t  * p_client_tk);

static void lls_timeout_handler(void * p_context)
{
    uint32_t err_code = ble_lls_c_alert_remove(p_lls_clients[m_timer_nbr]); //check this
	  APP_ERROR_CHECK(err_code);

}


/**@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num        Line number of the failing ASSERT call.
 * @param[in] p_file_name     File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Callback handling device manager events.
 *
 * @details This function is called to notify the application of device manager events.
 *
 * @param[in]   p_handle      Device Manager Handle. For link related events, this parameter
 *                            identifies the peer.
 * @param[in]   p_event       Pointer to the device manager event.
 * @param[in]   event_status  Status of the event.
 */
static ret_code_t device_manager_event_handler(const dm_handle_t * p_handle,
                                               const dm_event_t  * p_event,
                                               const ret_code_t    event_result)
{
    uint32_t err_code;

    switch (p_event->event_id)
    {
        case DM_EVT_CONNECTION:
					
            APPL_LOG("[APPL]:[0x%02X] << DM_EVT_CONNECTION\r\n", p_handle->connection_id);
						err_code = dm_security_setup_req((dm_handle_t *)p_handle);
            APP_ERROR_CHECK(err_code);
            break;
        case DM_EVT_DISCONNECTION:
            APPL_LOG("[APPL]:[0x%02X] >> DM_EVT_DISCONNECTION\r\n", p_handle->connection_id);

            if (m_peer_count == MAX_PEER_COUNT)
            {
                scan_start();
            }
            m_peer_count--;
						dm_device_delete( p_handle);
            APPL_LOG("[APPL]:[0x%02X] << DM_EVT_DISCONNECTION\r\n", p_handle->connection_id);
            break;

        case DM_EVT_SECURITY_SETUP:
            // Slave securtiy request received from peer, if from a non bonded device,
            // initiate security setup, else, wait for encryption to complete. Not used to avoid server overload.
				
						//err_code = dm_security_setup_req((dm_handle_t *)p_handle);
				    //APP_ERROR_CHECK(err_code);
            APPL_LOG("[APPL]:[0x%02X] << DM_EVT_SECURITY_SETUP\r\n", p_handle->connection_id);
            break;

        case DM_EVT_SECURITY_SETUP_COMPLETE:
            APPL_LOG("[APPL]:[0x%02X] >> DM_EVT_SECURITY_SETUP_COMPLETE, result 0x%08X. Conn id is %x\r\n",
                     p_handle->connection_id, event_result, p_handle->connection_id);
				    APP_ERROR_CHECK(event_result);
				
            // Discover peer's services.
            err_code = ble_db_discovery_start(&m_ble_db_discovery,
                                              p_event->event_param.p_gap_param->conn_handle);
            APP_ERROR_CHECK(err_code);
            m_peer_count++;
            if (m_peer_count < MAX_PEER_COUNT)
            {
                scan_start();
            }
						
            break;

        case DM_EVT_LINK_SECURED:
            APPL_LOG("[APPL]:[0x%02X] >> DM_EVT_LINK_SECURED, result 0x%08X\r\n",
                     p_handle->connection_id, event_result);
				    APP_ERROR_CHECK(event_result);
            break;

        case DM_EVT_DEVICE_CONTEXT_LOADED:
            APPL_LOG("[APPL]:[0x%02X] >> DM_EVT_DEVICE_CONTEXT_LOADED\r\n", p_handle->connection_id);
            APP_ERROR_CHECK(event_result);
            break;

        case DM_EVT_DEVICE_CONTEXT_STORED:
            APPL_LOG("[APPL]:[0x%02X] >> DM_EVT_DEVICE_CONTEXT_STORED\r\n", p_handle->connection_id);
            APP_ERROR_CHECK(event_result);
            break;

        case DM_EVT_DEVICE_CONTEXT_DELETED:
            APPL_LOG("[APPL]:[0x%02X] >> DM_EVT_DEVICE_CONTEXT_DELETED\r\n",
                     p_handle->connection_id);
            APP_ERROR_CHECK(event_result);
            break;

        default:
            break;
    }

    return NRF_SUCCESS;
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t         err_code;
    uint8_t client_nbr;

    switch (p_ble_evt->header.evt_id)
    {

        case BLE_GAP_EVT_ADV_REPORT:
				m_peer_addr_actual = p_ble_evt->evt.gap_evt.params.adv_report.peer_addr;

            if (client_get(&m_peer_addr_actual, &client_nbr)!=NRF_SUCCESS)
            {
               // APPL_LOG("[APPL]: Cannot find a client with this address.\r\n");
							  return;
						}
						
						err_code = sd_ble_gap_connect(&p_ble_evt->evt.gap_evt.params.adv_report. \
																					peer_addr,
																					&m_scan_param,
																					&m_connection_param);

						if (err_code != NRF_SUCCESS)
						{
								APPL_LOG("[APPL]: Connection Request Failed, reason %d\r\n", err_code);
						}
						APPL_LOG("[APPL]: BLE_GAP_EVT_ADV_REPORT. Try connecting with device.\r\n");
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                APPL_LOG("[APPL]: Scan Timeout.\r\n");
            }
            else if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                APPL_LOG("[APPL]: Connection Request Timeout.\r\n");
            }
            break;
				case BLE_GAP_EVT_DISCONNECTED:
						APPL_LOG("[APPL]:BLE_GAP_EVT_DISCONNECTED\r\n");
				  	break;
				case BLE_GAP_EVT_AUTH_KEY_REQUEST: 
					
						APPL_LOG("[APPL]:BLE_GAP_EVT_AUTH_KEY_REQUEST\r\n");

						if (client_get(&m_peer_addr_actual, &client_nbr)!=NRF_SUCCESS)
            {
                //APPL_LOG("[APPL]: Cannot find a client with this address.\r\n");
							  return;
						}
						err_code = sd_ble_gap_auth_key_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_AUTH_KEY_TYPE_OOB, p_whitelist_tk[client_nbr]->tk);
						break;
        default:
            break;
    }
}


/**@brief Function for handling the Application's system events.
 *
 * @param[in]   sys_evt   system event.
 */
static void on_sys_evt(uint32_t sys_evt)
{
    switch (sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
        case NRF_EVT_FLASH_OPERATION_ERROR:
            if (m_memory_access_in_progress)
            {
                m_memory_access_in_progress = false;
                scan_start();
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 *  been received.
 *
 * @param[in] p_ble_evt     Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);

    for (uint8_t i = 0; i < MAX_PEER_COUNT; i++)
        if (p_lls_clients[i]->conn_handle == p_ble_evt->evt.common_evt.conn_handle)
        {
            ble_lls_c_on_ble_evt(p_lls_clients[i], p_ble_evt);
            break;
        }

    on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
#ifdef S130
    ble_enable_params.gatts_enable_params.attr_tab_size = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
#endif
    ble_enable_params.gatts_enable_params.service_changed = false;
#ifdef S120
    ble_enable_params.gap_enable_params.role = BLE_GAP_ROLE_CENTRAL;
#endif

    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for System events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Device Manager.
 *
 * @details Device manager is initialized here.
 */
static void device_manager_init(void)
{
    dm_init_param_t        init_param;
    dm_application_param_t param;
    uint32_t err_code;

    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    init_param.clear_persistent_data = false;

#ifdef BOND_DELETE_ALL_BUTTON_PIN
    // Clear all bonded devices if user requests to.
    init_param.clear_persistent_data =
        ((nrf_gpio_pin_read(BOND_DELETE_ALL_BUTTON_PIN) == 0) ? true : false);
#endif

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Event handler to be registered with the module.
    param.evt_handler = device_manager_event_handler;

    // Service or protocol context for device manager to load, store and apply on behalf of application.
    // Here set to client as application is a GATT client.
    param.service_type = DM_PROTOCOL_CNTXT_GATT_CLI_ID;

    // Secuirty parameters to be used for security procedures.
    param.sec_param.bond             = SEC_PARAM_BOND;
    param.sec_param.mitm             = SEC_PARAM_MITM;
    param.sec_param.io_caps          = SEC_PARAM_IO_CAPABILITIES;
    param.sec_param.oob              = SEC_PARAM_OOB;
    param.sec_param.min_key_size     = SEC_PARAM_MIN_KEY_SIZE;
    param.sec_param.max_key_size     = SEC_PARAM_MAX_KEY_SIZE;
    param.sec_param.kdist_periph.enc = 1;
    param.sec_param.kdist_periph.id  = 1;

    err_code = dm_register(&m_dm_app_id, &param);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Signals alert event from Immediate Alert or Link Loss services.
 *
 * @param[in] alert_level  Requested alert level.
 */
static void alert_signal(uint8_t alert_level)
{
    uint32_t err_code;
	
    switch (alert_level)
    {
        case BLE_CHAR_ALERT_LEVEL_NO_ALERT:
						LEDS_OFF(LEDS_MASK);
						nrf_delay_ms(1000);
            err_code = bsp_indication_set(BSP_INDICATE_ALERT_OFF);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_CHAR_ALERT_LEVEL_MILD_ALERT:
            err_code = bsp_indication_set(BSP_INDICATE_ALERT_0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_CHAR_ALERT_LEVEL_HIGH_ALERT:
						LEDS_ON(LEDS_MASK);
						nrf_delay_ms(1000);
            err_code = bsp_indication_set(BSP_INDICATE_ALERT_3);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Link Loss Service Client Handler.
 */
static void lls_c_evt_handler(ble_lls_c_t * p_lls_c, ble_lls_c_evt_t * p_lls_c_evt)
{
    uint8_t  i;
	uint32_t err_code;

		//This alert continues until one of following conditions occurs:
		//->an implementation-specific timeout
    //->user interaction on this device
    //->the physical link is reconnected.

    switch (p_lls_c_evt->evt_type)
    {
        case BLE_LLS_C_EVT_DISCOVERY_COMPLETE:
					err_code = ble_lls_c_alert_set(p_lls_c);
					APP_ERROR_CHECK(err_code);
					  //alert_signal(p_lls_c->alert_value);
				
            for (i = 0; i < MAX_PEER_COUNT; i++)
            {
                if (p_lls_c->conn_handle == p_lls_clients[i]->conn_handle)
                {
										uint32_t err_code = app_timer_create(&m_lls_timer[i],
																					 APP_TIMER_MODE_SINGLE_SHOT,
																					 lls_timeout_handler);
                    APP_ERROR_CHECK(err_code);
                    err_code    = app_timer_start(m_lls_timer[i], LLS_INTERVAL, NULL);
                    APP_ERROR_CHECK(err_code);
									  APPL_LOG("[APPL]: Timer is starting\r\n");
									  m_timer_nbr = i;
                    break;
                }
            }
						
            break;
				
				case BLE_LLS_C_EVT_LINK_LOSS_DEVICE_DISCONNECT:
					   alert_signal(p_lls_c->alert_value);
				
						 //remove client from p_lls_clients :)
						 p_lls_c->conn_handle     = BLE_CONN_HANDLE_INVALID;
						 p_lls_c->lls_cccd_handle = BLE_GATT_HANDLE_INVALID;
			       p_lls_c->alert_value     = BLE_CHAR_ALERT_LEVEL_NO_ALERT;
					   break;
				
        default:
            break;
    }
}


/**
 * @brief Functions initializes a Link Loss Service client module.
 */
void lls_c_init(ble_lls_c_t *  m_lls_client)
{

    for (uint8_t i = 0; i < MAX_PEER_COUNT; i++)
	  {
        p_lls_clients[i] = m_lls_client + i;
		}

    ble_lls_c_init_t lls_c_init_obj;
    lls_c_init_obj.evt_handler = lls_c_evt_handler;

    uint32_t err_code = ble_lls_c_init(p_lls_clients, MAX_PEER_COUNT, &lls_c_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
#ifdef BOND_DELETE_ALL_BUTTON_PIN
    // Set Wakeup and Bonds Delete buttons as wakeup sources.
    nrf_gpio_cfg_sense_input(BOND_DELETE_ALL_BUTTON_PIN,
                             BUTTON_PULL,
                             NRF_GPIO_PIN_SENSE_LOW);
#endif
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();

    APP_ERROR_CHECK(err_code);
}


/**@breif Function to start scanning.
 */
static void scan_start(void)
{
    uint32_t err_code;
    uint32_t count;

    // Verify if there is any flash access pending, if yes delay starting scanning until
    // it's complete.
    err_code = pstorage_access_status_get(&count);
    APP_ERROR_CHECK(err_code);

    if (count != 0)
    {
        m_memory_access_in_progress = true;
        return;
    }

    // No devices in whitelist, hence non selective performed.
    m_scan_param.active      = 0;             // Active scanning set.
    m_scan_param.selective   = 0;             // Selective scanning not set.
    m_scan_param.interval    = SCAN_INTERVAL; // Scan interval.
    m_scan_param.window      = SCAN_WINDOW;   // Scan window.
    m_scan_param.p_whitelist = NULL;          // No whitelist provided.
    m_scan_param.timeout     = 0x0000;        // No timeout.

    err_code = sd_ble_gap_scan_start(&m_scan_param);
    APP_ERROR_CHECK(err_code);
}



/**
 * @brief Database discovery collector initialization.
 */
static void db_discovery_init(void)
{
    uint32_t err_code = ble_db_discovery_init();

    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Function creates clients database.
 */

static uint32_t client_database_add_client(ble_gap_addr_t          * p_client_addr,
                                           ble_advdata_tk_value_t  * p_client_tk)
{
    if (m_whitelist_actual_client_nbr > MAX_PEER_COUNT - 1 )
    {
        APPL_LOG("[APPL]: Too many devices in whitelist. \r\n");
        return NRF_ERROR_DATA_SIZE;
    }
		
    p_whitelist_addr[m_whitelist_actual_client_nbr] = p_client_addr;
    p_whitelist_tk[m_whitelist_actual_client_nbr]  = p_client_tk;

		APPL_LOG("[APPL]: Add client with address: [%02X %02X %02X %02X %02X %02X]. \r\n whitelist actual nbr %x\r\n",
				 p_client_addr->addr[0], p_client_addr->addr[1], p_client_addr->addr[2],
				 p_client_addr->addr[3], p_client_addr->addr[4], p_client_addr->addr[5],
				 m_whitelist_actual_client_nbr);

		m_whitelist_actual_client_nbr++;
		
		return NRF_SUCCESS;
}

/*
 * @brief Function searchs local database  client with specified address.
 * @retval Index of searched client or NULL if no client has been found.
 */

static uint8_t client_get(ble_gap_addr_t * p_client_addr, uint8_t * p_client_nbr)
{
    for (uint8_t i = 0; i < m_whitelist_actual_client_nbr; i++)
    {
			//if(memcmp(p_client_addr->addr, p_whitelist_addr[i]->addr, BLE_GAP_ADDR_LEN) ==0)
        if (memcmp(p_client_addr, p_whitelist_addr[i], BLE_GAP_ADDR_LEN) == 0) //consider that it compare whole address with address type.
        {
            APPL_LOG("[APPL]: Found client with address: [%02X %02X %02X %02X %02X %02X]. His number is %x\r\n",
                     p_client_addr->addr[0], p_client_addr->addr[1], p_client_addr->addr[2],
                     p_client_addr->addr[3], p_client_addr->addr[4], p_client_addr->addr[5], i);

					  *p_client_nbr=i;
            return NRF_SUCCESS;
        }
    }

    return NRF_ERROR_NOT_FOUND;
}

//dodaj usuwanie p_client z database.
static uint32_t client_database_remove_client(ble_gap_addr_t * p_client_addr)
{
    uint8_t client_nbr;
	  uint32_t err_code = client_get(p_client_addr, &client_nbr);

    if (err_code != NRF_SUCCESS)
    {
        APPL_LOG("[APPL]: Cannot find client in database. ");
        return err_code;
    }

    for (; client_nbr < (MAX_PEER_COUNT - 1); client_nbr++)
    {
        p_whitelist_addr[client_nbr] = p_whitelist_addr[client_nbr + 1];
        p_whitelist_tk[client_nbr]  = p_whitelist_tk[client_nbr + 1];
    }

    m_whitelist_actual_client_nbr--;

    return NRF_SUCCESS;
}

		// check if client exists in database. then java app decides to connect or not.  
	  // If yes check hi_password, then get tk_value, add client to database and try connect. Checks error code.



static uint32_t java_client_get(void)
{


}

		// check client in java app.
		// if we want to disconnect: check bye_password, remove client from database,
		// remove alert, then return success. 
	
static uint32_t java_client_remove(void)
{

}
/**@brief Function for application main entry. */
int main(void)
{
	  ble_lls_c_t   m_lls_client[MAX_PEER_COUNT];
	
    // Initialization of various modules.
    app_trace_init();
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);
    buttons_init();
		ble_stack_init();
    device_manager_init();
    db_discovery_init();
    lls_c_init(m_lls_client);


    // Initialize timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);


    ble_gap_addr_t client_addr[3];
    ble_advdata_tk_value_t  client_tk = OOB_AUTH_KEY;

    client_addr[0].addr[0] = 0x44;
    client_addr[0].addr[1] = 0x3E;
    client_addr[0].addr[2] = 0xDC;
    client_addr[0].addr[3] = 0x81;
    client_addr[0].addr[4] = 0x40;
    client_addr[0].addr[5] = 0xCA;
    client_addr[0].addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
		
    uint32_t err_code = client_database_add_client(&client_addr[0], &client_tk);
    APP_ERROR_CHECK(err_code);
		
		client_addr[1].addr[0] = 0xDA;
    client_addr[1].addr[1] = 0x2F;
    client_addr[1].addr[2] = 0x30;
    client_addr[1].addr[3] = 0x28;
    client_addr[1].addr[4] = 0x95;
    client_addr[1].addr[5] = 0xCE;
		client_addr[1].addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
		
    err_code = client_database_add_client(&client_addr[1], &client_tk);
    APP_ERROR_CHECK(err_code);
				
		client_addr[2].addr[0] = 0x44;
    client_addr[2].addr[1] = 0x3E;
    client_addr[2].addr[2] = 0xDC;
    client_addr[2].addr[3] = 0x81;
    client_addr[2].addr[4] = 0x40;
    client_addr[2].addr[5] = 0xCA;
		client_addr[2].addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;

    err_code = client_database_add_client(&client_addr[2], &client_tk);
    APP_ERROR_CHECK(err_code);
		
		printf("Restart!\r\n\r\n");

    // Start scanning for devices.
    scan_start();

    for (;;)
    {
        power_manage();
    }
}


