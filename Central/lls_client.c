#include "lls_client.h"

#define LOG app_trace_log

#define WRITE_MESSAGE_LENGTH   	2    					/**< Length of the write message for CCCD. */
#define MAX_CLIENTS  						DEVICE_MANAGER_MAX_CONNECTIONS  /**< Max number of clients. */
#define TX_BUFFER_MASK         	0x07                  					/**< TX Buffer mask, must be a mask of continuous zeroes, followed by continuous sequence of ones: 000...111. */
#define TX_BUFFER_SIZE         	(TX_BUFFER_MASK + 1)  					/**< Size of send buffer, which is 1 higher than the mask. */
#define HRM_FLAG_MASK_HR_16BIT (0x01 << 0)           /**< Bit mask used to extract the type of heart rate value. This is used to find if the received heart rate is a 16 bit value or an 8 bit value. */

static ble_lls_c_t ** mp_ble_lls_c;                /**< Pointer to pointer to the current instances of the LLS Client module. The memory for this provided by the application.*/
static uint8_t max_clients;											   /**< Max clients number. */


/**
 * @brief Function for performing a Read procedure.
 *
 * @param[in]   conn_handle    Handle of the connection on which to perform the write operation.
 * @param[in]   read_handle    Handle of the attribute to be written.
 * @param[in]   offset         Offset of data to be written.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t read_characteristic_value(uint16_t conn_handle, uint16_t read_handle, uint8_t offset)
{
		return sd_ble_gattc_read(conn_handle, read_handle,offset);
}

/**@brief Function for performing a Write procedure.
 *
 * @param[in]   conn_handle    Handle of the connection on which to perform the write operation.
 * @param[in]   write_handle   Handle of the attribute to be written.
 * @param[in]   length         Length of data to be written.
 * @param[in]   p_value        Data to be written.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
 static uint32_t write_characteristic_value(uint16_t  conn_handle,
																						uint16_t  write_handle,
																						uint8_t  length,
																						uint8_t * p_value)
{
    ble_gattc_write_params_t write_params;

    memset(&write_params, 0, sizeof(write_params));

    write_params.handle     = write_handle;
    write_params.write_op   = BLE_GATT_OP_WRITE_REQ;
    write_params.offset     = 0;
    write_params.len        = length;
    write_params.p_value    = p_value;

    return sd_ble_gattc_write(conn_handle, &write_params);
}

/**
 * @brief Function for handling the Disconnect event.
 *
 * @param[in]   p_ble_lls_c  Link Loss Service structure.
 * @param[in]   p_ble_evt    Event received from the BLE stack.
 */
static void on_disconnect(ble_lls_c_t * p_ble_lls_c, const ble_evt_t * p_ble_evt)
{
    uint8_t reason = p_ble_evt->evt.gap_evt.params.disconnected.reason;
	
	  if (reason == BLE_HCI_CONNECTION_TIMEOUT)
			{
					printf("rozlaczam\r\n");
				
					ble_lls_c_evt_t evt;
					evt.evt_type = BLE_LLS_C_EVT_LINK_LOSS_ALERT;
				
          p_ble_lls_c->evt_handler(p_ble_lls_c, &evt);
			}
}


/**@brief     Function for handling events from the database discovery module.
 *
 * @details   This function will handle an event from the database discovery module, and determine
 *            if it relates to the discovery of link loss service at the peer. If so, it will
 *            call the application's event handler indicating that the link loss service has been
 *            discovered at the peer. It also populates the event with the service related
 *            information before providing it to the application.
 *
 * @param[in] p_evt Pointer to the event received from the database discovery module.
 *
 */
static void db_discovery_evt_handler(ble_db_discovery_evt_t * p_evt)
{  
     uint8_t lls_c_nbr = 0;   // Actual client number.

		// Check if the Link Loss Service was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_LINK_LOSS_SERVICE &&
        p_evt->params.discovered_db.srv_uuid.type == BLE_UUID_TYPE_BLE)
    {
				//Strore the pointer.
				for(uint8_t i=0; i<MAX_CLIENTS; i++)
				{
							if(mp_ble_lls_c[i]->conn_handle == BLE_CONN_HANDLE_INVALID) //find the first available client 
							{
									mp_ble_lls_c[i]->conn_handle = p_evt->conn_handle;
								  lls_c_nbr = i; //it stores actual client nbr.
									break;
							}
							else if(i==MAX_CLIENTS-1) // it should not occur. But it means that the clients memory is exceeded.
							{
								LOG("[LLS_C]: Cannot connect. The maximum nbr of connected clients is 8. \r\n");
									return;	
							}
				}
        
        // Find the CCCD Handle of the Link Loss Measurement characteristic.
        for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                BLE_UUID_ALERT_LEVEL_CHAR)
            {
							  // Found Alert Level characteristic. Store CCCD handle, client number increase and break.
							  LOG("[LLS_C]: Found Alert Level characteristic.\r\n");
							
							  if(lls_c_nbr<max_clients)
								{
									  mp_ble_lls_c[lls_c_nbr]->lls_cccd_handle =
                    p_evt->params.discovered_db.charateristics[i].cccd_handle;
										mp_ble_lls_c[lls_c_nbr]->llm_value_handle      =
                    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
									//	lls_c_nbr++;
								}
								else //Size of table is exceeded. It should not occur. 
								{
									LOG("[LLS_C]: Size of Link Loss Client table is exceeded.\r\n");
										return;
								}
                break;
            }
        }

        LOG("[LLS_C]: Characteristic discovered at peer.\r\n");

        ble_lls_c_evt_t evt;

        evt.evt_type = BLE_LLS_C_EVT_DISCOVERY_COMPLETE;

        mp_ble_lls_c[lls_c_nbr]->evt_handler(mp_ble_lls_c[lls_c_nbr], &evt);
    }

}


/***********************************************************************************
															PUBLIC API
************************************************************************************/

uint32_t ble_lls_c_init(ble_lls_c_t **     pp_ble_lls_c,
                        uint8_t            ble_lls_c_size,
                        ble_lls_c_init_t * p_ble_lls_c_init)
{
	
		if(ble_lls_c_size==0)
		{
			return NRF_ERROR_INVALID_PARAM;
		}
		
		//Initialize a structure contains all clients.
	  max_clients  = ble_lls_c_size;
	  mp_ble_lls_c = pp_ble_lls_c;
		
		for(uint8_t i=0; i< max_clients; i++)
		{
        mp_ble_lls_c[i]->evt_handler     = p_ble_lls_c_init->evt_handler;
        mp_ble_lls_c[i]->conn_handle     = BLE_CONN_HANDLE_INVALID;
        mp_ble_lls_c[i]->lls_cccd_handle = BLE_GATT_HANDLE_INVALID;
			  mp_ble_lls_c[i]->alert_value     = BLE_CHAR_ALERT_LEVEL_NO_ALERT;
		}
			
    // Register with discovery module for the discovery of the service.
    ble_uuid_t uuid;

    uuid.type = BLE_UUID_TYPE_BLE;
    uuid.uuid = BLE_UUID_LINK_LOSS_SERVICE;

    return ble_db_discovery_evt_register(&uuid,
                                             db_discovery_evt_handler);	
}

void ble_lls_c_on_ble_evt(ble_lls_c_t * p_ble_lls_c, const ble_evt_t * p_ble_evt)
{
	printf("event id is %x\r\n", p_ble_evt->header.evt_id);

	ble_gattc_evt_write_rsp_t gatt_client = p_ble_evt->evt.gattc_evt.params.write_rsp;
	
    if ((p_ble_lls_c == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            break;
				case BLE_GATTC_EVT_WRITE_RSP:
					LOG("[LLS_C]: GATT client handle is %x. Write answer: %x %x %x %x %x. CODE is %x\r\n", p_ble_lls_c->conn_handle, *gatt_client.data, gatt_client.handle, gatt_client.len, 
																				gatt_client.offset, gatt_client.write_op, p_ble_evt->evt.gattc_evt.gatt_status);
						break;
				 case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_ble_lls_c, p_ble_evt);
					  break;
				 case BLE_GAP_EVT_CONN_SEC_UPDATE:
				 {
					 ble_lls_c_evt_t evt;
           evt.evt_type = BLE_LLS_C_EVT_LINK_LOSS_SET;
          p_ble_lls_c->evt_handler(p_ble_lls_c, &evt);
				 }
				 break;
				 case BLE_GATTC_EVT_CHAR_DISC_RSP:
					 printf("AAAAAAA\r\n\r\n");
				 break;
        default:
            break;
    }
}


 uint32_t ble_lls_c_alert_set(ble_lls_c_t * p_ble_lls_c)
{
    if (p_ble_lls_c == NULL)
    {
        return NRF_ERROR_NULL;
    }	
		
		 uint8_t alert_value      = BLE_CHAR_ALERT_LEVEL_HIGH_ALERT;
     p_ble_lls_c->alert_value = BLE_CHAR_ALERT_LEVEL_HIGH_ALERT;
		
		 return write_characteristic_value(p_ble_lls_c->conn_handle,
														p_ble_lls_c->llm_value_handle, sizeof(uint8_t),&alert_value);	
}


 uint32_t ble_lls_c_alert_remove(ble_lls_c_t * p_ble_lls_c)
{
    if (p_ble_lls_c == NULL)
    {
        return NRF_ERROR_NULL;
    }	
		
		uint8_t alert_value      = BLE_CHAR_ALERT_LEVEL_NO_ALERT;
    p_ble_lls_c->alert_value = BLE_CHAR_ALERT_LEVEL_NO_ALERT;
		
		return write_characteristic_value(p_ble_lls_c->conn_handle, 
													p_ble_lls_c->llm_value_handle, sizeof(uint8_t),&alert_value);
}

