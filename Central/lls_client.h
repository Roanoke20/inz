
#ifndef LLS_CLIENT_H__
#define LLS_CLIENT_H__


#include <string.h>
#include <stdbool.h>
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "app_trace.h"
#include "ble_db_discovery.h"
#include "ble_srv_common.h"
#include "boards.h"
#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "device_manager.h"

/**@brief LLS Client event type. */
typedef enum
{
    BLE_LLS_C_EVT_DISCOVERY_COMPLETE = 1,  /**< Event indicating that the Link Loss Service has been discovered at the peer. */
    BLE_LLS_C_EVT_ALERT_SET,							 /**< Event indicating that bonding is over and alert should be set. */
		BLE_LLS_C_EVT_ALERT_REMOVE,						 /**< Event indicating that device will disconnect in normal way. */
		BLE_LLS_C_EVT_SERVICE_NOT_FOUND				 /**< Event indicating that the Link Loss Service has not been found at the peer. */
} ble_lls_c_evt_type_t;


/** @brief Forward declaration of the ble_bas_t type. */
typedef struct ble_lls_c_s ble_lls_c_t;


/**@brief Structure containing the link loss alert level received from the peer. */
typedef struct
{
    uint16_t lls_value;  /**< Link Loss Value. */
} ble_llm_t;


/**@brief Link Loss Event structure. */
typedef struct
{
    ble_lls_c_evt_type_t evt_type;  /**< Type of the event. */
    union
    {
        ble_llm_t lls;  /**< Link Loss measurement received. This will be filled if the evt_type is @ref BLE_LLS_C_EVT_LLS_NOTIFICATION. */
    } params;
} ble_lls_c_evt_t;


/**@brief   Event handler type.
 *
 * @details This is the type of the event handler that should be provided by the application
 *          of this module in order to receive events.
 */
typedef void (* ble_lls_c_evt_handler_t) (ble_lls_c_t * p_ble_lls_c,  ble_lls_c_evt_t * p_evt);

/** @brief Link Loss Client Structure. */
struct ble_lls_c_s
{
		uint16_t                 conn_handle;      /**< Connection handle as provided by the SoftDevice. */
    uint16_t                 lls_cccd_handle;  /**< Handle of the CCCD of the Link Loss Measurement characteristic. */
    uint16_t                 llm_value_handle; /**< Handle of the Link Loss Measurement characteristic as provided by the SoftDevice. */
    ble_lls_c_evt_handler_t  evt_handler;      /**< Application event handler to be called when there is an event related to the heart rate service. */ 
};


/**@brief Link Loss Client initialization structure.
 */
typedef struct
{
    ble_lls_c_evt_handler_t evt_handler;  /**< Event handler to be called by the Heart Rate Client module whenever there is an event related to the Heart Rate Service. */
} ble_lls_c_init_t;



/**@brief     Function for initializing the link loss client module.
 *
 * @details   This function will register with the DB Discovery module. There it
 *            registers for the Link Loss Service. Doing so will make the DB Discovery
 *            module look for the presence of a Link Loss Service instance at the peer when a
 *            discovery is started.
 *
 * @param[in] pp_ble_lls_c      Pointer to pointer to the link loss client structures.
 * @param[in] p_ble_lls_c_size Size of the link loss client structure.
 * @param[in] p_ble_hrs_c_init Pointer to the link loss initialization structure containing the
 *                             initialization information.
 *
 * @retval    NRF_SUCCESS On successful initialization. Otherwise an error code. This function
 *                        propagates the error code returned by the Database Discovery module API
 *                        @ref ble_db_discovery_evt_register.
 */
uint32_t ble_lls_c_init( ble_lls_c_t ** pp_ble_lls_c, uint8_t p_ble_lls_c_size, ble_lls_c_init_t * p_ble_lls_c_init);


/**@brief   Function for configuartion of Alert Level in Link Loss Service at the peer.
 *
 * @details This function will sets Alert Level as �High Alert� at the peer
 *          by writing to the the Link Loss Measurement Characteristic.
 *          To do this uses the GATT Characteristic Read Value procedure.
 *
 * @param   p_ble_lls_c Pointer to the link loss client structure.
 *
 * @retval  NRF_SUCCESS If the SoftDevice has been requested to write to the CCCD of the peer.
 *                      Otherwise, an error code. This function propagates the error code returned 
 *                      by the SoftDevice API @ref sd_ble_gattc_write.
 */
 uint32_t ble_lls_c_alert_set(ble_lls_c_t * p_ble_lls_c);


/**@brief   Function for configuartion of Alert Level in Link Loss Service at the peer.
 *
 * @details This function will set Alert Level as �No Alert� at the peer
 *          by writing to the the Link Loss Measurement Characteristic.
 *          To do this uses the GATT Characteristic Read Value procedure.
 *
 * @param   p_ble_lls_c Pointer to the link loss client structure.
 *
 * @retval  NRF_SUCCESS If the SoftDevice has been requested to write to the CCCD of the peer.
 *                      Otherwise, an error code. This function propagates the error code returned 
 *                      by the SoftDevice API @ref sd_ble_gattc_write.
 */
 uint32_t ble_lls_c_alert_remove(ble_lls_c_t * p_ble_lls_c);

/**@brief     Function for handling BLE events from the SoftDevice.
 *
 * @details   This function will handle the BLE events received from the SoftDevice. If a BLE
 *            event is relevant to the Heart Rate Client module, then it uses it to update
 *            interval variables and, if necessary, send events to the application.
 *
 * @param[in] p_ble_hrs_c Pointer to the heart rate client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event.
 */
void ble_lls_c_on_ble_evt(ble_lls_c_t * p_ble_lls_c, const ble_evt_t * p_ble_evt);

/**@brief   Function for requesting the peer to start sending notification of Heart Rate
 *          Measurement.
 *
 * @details This function will enable to notification of the Heart Rate Measurement at the peer
 *          by writing to the CCCD of the Heart Rate Measurement Characteristic.
 *
 * @param   p_ble_hrs_c Pointer to the heart rate client structure.
 *
 * @retval  NRF_SUCCESS If the SoftDevice has been requested to write to the CCCD of the peer.
 *                      Otherwise, an error code. This function propagates the error code returned 
 *                      by the SoftDevice API @ref sd_ble_gattc_write.
 */
//uint32_t ble_lls_c_llm_notif_enable(ble_lls_c_t * p_ble_lls_c);

//uint32_t client_handling_destroy(const dm_handle_t * p_handle);
	/**@brief Function for performing a Write procedure.
 *
 * @param[in]   conn_handle    Handle of the connection on which to perform the write operation.
 * @param[in]   write_handle   Handle of the attribute to be written.
 * @param[in]   length         Length of data to be written.
 * @param[in]   p_value        Data to be written.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 
 uint32_t write_characteristic_value(uint16_t  conn_handle,
                                           uint16_t  write_handle,
                                           uint8_t  length,
                                           uint8_t * p_value);
*/																						
#endif

