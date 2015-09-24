#include "lls_client.h"
#include "nrf_delay.h"

#define LOG app_trace_log

#define WRITE_MESSAGE_LENGTH   	2    					/**< Length of the write message for CCCD. */
#define MAX_CLIENTS  						DEVICE_MANAGER_MAX_CONNECTIONS  /**< Max number of clients. */
#define TX_BUFFER_MASK         	0x07                  					/**< TX Buffer mask, must be a mask of continuous zeroes, followed by continuous sequence of ones: 000...111. */
#define TX_BUFFER_SIZE         	(TX_BUFFER_MASK + 1)  					/**< Size of send buffer, which is 1 higher than the mask. */
#define HRM_FLAG_MASK_HR_16BIT (0x01 << 0)           /**< Bit mask used to extract the type of heart rate value. This is used to find if the received heart rate is a 16 bit value or an 8 bit value. */


/**@brief Client states. */
typedef enum
{
    IDLE,                                           /**< Idle state. */
    STATE_SERVICE_DISC,                             /**< Service discovery state. */
    STATE_ALERT_SENT,//STATE_NOTIF_ENABLE,          /**< State where the request to enable notifications is sent to the peer. . */
    STATE_RUNNING,                                  /**< Running state. */
    STATE_ERROR                                     /**< Error state. */
} client_state_t;


/**@brief Client context information. */
typedef struct
{
    ble_db_discovery_t           srv_db;            /**< The DB Discovery module instance associated with this client. */
    dm_handle_t                  handle;            /**< Device manager identifier for the device. */
    uint8_t                      char_index;        /**< Client characteristics index in discovered service information. */
    uint8_t                      state;             /**< Client state. */
} client_t;


typedef enum
{
    READ_REQ,  /**< Type identifying that this tx_message is a read request. */
    WRITE_REQ  /**< Type identifying that this tx_message is a write request. */
} tx_request_t;


/**@brief Structure for writing a message to the peer, i.e. CCCD. */
typedef struct
{
    uint8_t                  gattc_value[WRITE_MESSAGE_LENGTH];  /**< The message to write. */
    ble_gattc_write_params_t gattc_params;                       /**< GATTC parameters for this message. */
} write_params_t;


/**@brief Structure for holding data to be transmitted to the connected central. */
typedef struct
{
    uint16_t     conn_handle;  /**< Connection handle to be used when transmitting this message. */
    tx_request_t type;         /**< Type of this message, i.e. read or write message. */
    union
    {
        uint16_t       read_handle;  /**< Read request message. */
        write_params_t write_req;    /**< Write request message. */
    } req;
} tx_message_t;



//static client_t         m_client[MAX_CLIENTS];      /**< Client context information list. */
//static uint8_t          m_client_count;             /**< Number of clients. */


static ble_lls_c_t ** mp_ble_lls_c;                /**< Pointer to pointer to the current instances of the LLS Client module. The memory for this provided by the application.*/
static uint8_t max_clients;											   /**< Max clients number. */
static tx_message_t  m_tx_buffer[TX_BUFFER_SIZE];  /**< Transmit buffer for messages to be transmitted to the central. */
static uint32_t      m_tx_insert_index = 0;        /**< Current index in the transmit buffer where the next message should be inserted. */
static uint32_t      m_tx_index = 0;               /**< Current index in the transmit buffer from where the next message to be transmitted resides. */

static uint8_t alert_value=BLE_CHAR_ALERT_LEVEL_HIGH_ALERT;
	

/**@brief Function for passing any pending request from the buffer to the stack.
 */
static void tx_buffer_process(void)
{
    if (m_tx_index != m_tx_insert_index)
    {
        uint32_t err_code;

        if (m_tx_buffer[m_tx_index].type == READ_REQ)
        {
            err_code = sd_ble_gattc_read(m_tx_buffer[m_tx_index].conn_handle,
                                         m_tx_buffer[m_tx_index].req.read_handle,
                                         0);
        }
        else
        {
            err_code = sd_ble_gattc_write(m_tx_buffer[m_tx_index].conn_handle,
                                          &m_tx_buffer[m_tx_index].req.write_req.gattc_params);
        }
        if (err_code == NRF_SUCCESS)
        {
            LOG("[LLS_C]: SD Read/Write API returns Success..\r\n");
            m_tx_index++;
            m_tx_index &= TX_BUFFER_MASK;
        }
        else
        {
            LOG("[LLS_C]: SD Read/Write API returns error. This message sending will be "
                "attempted again..Error code is=%x\r\n ", err_code);
        }
    }
}

/**@brief Function for creating a message for writing to the CCCD.
 */
static uint32_t cccd_configure(uint16_t conn_handle, uint16_t handle_cccd , bool enable )
{
    LOG("[LLS_C]: Configuring CCCD. CCCD Handle = %d, Connection Handle = %d\r\n",
        handle_cccd,conn_handle);

    tx_message_t * p_msg;
    uint16_t       cccd_val = enable ? BLE_GATT_HVX_NOTIFICATION : 0;

    p_msg              = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index &= TX_BUFFER_MASK;

    p_msg->req.write_req.gattc_params.handle   = handle_cccd;
    p_msg->req.write_req.gattc_params.len      = WRITE_MESSAGE_LENGTH;
    p_msg->req.write_req.gattc_params.p_value  = p_msg->req.write_req.gattc_value;
    p_msg->req.write_req.gattc_params.offset   = 0;
    p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;
    p_msg->req.write_req.gattc_value[0]        = LSB(cccd_val);
    p_msg->req.write_req.gattc_value[1]        = MSB(cccd_val);
    p_msg->conn_handle                         = conn_handle;
    p_msg->type                                = WRITE_REQ;

    tx_buffer_process();
    return NRF_SUCCESS;
}


/**
 * @brief Function set value in the Link Loss Characteristic (Alert Level Updating).
 *
 * @param[in] p_ble_lls_c Pointer to the structure, which value will be updated.
 * @param[in] alert_value Value of alert, wich will be set.

static void ble_lls_c_service_set_value(ble_lls_c_t * p_ble_lls_c)
{
		tx_message_t * p_msg;

    if (m_client_state != STATE_RUNNING)
    {
        return NRF_ERROR_INVALID_STATE;
    }


    p_msg              = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index &= TX_BUFFER_MASK;

    p_msg->req.write_req.gattc_params.handle   = p_ble_lls_c->llm_value_handle;
    p_msg->req.write_req.gattc_params.len      = sizeof(alert_value);
   // p_msg->req.write_req.gattc_params.p_value  = p_msg->req.write_req.gattc_value;
    p_msg->req.write_req.gattc_params.offset   = 0;
    p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_CMD;
    p_msg->req.write_req.gattc_value[0]        = LSB(alert_value);
    p_msg->req.write_req.gattc_value[1]        = MSB(alert_value);
    p_msg->conn_handle                         = p_ble_lls_c->conn_handle;
    p_msg->type                                = WRITE_REQ;

		tx_buffer_process();
}

		*/

void ble_lls_c_disconnect(ble_lls_c_t * p_ble_lls_c)
{
   ble_lls_c_evt_t evt;

   evt.evt_type = BLE_LLS_C_EVT_ALERT_REMOVE;
   p_ble_lls_c->evt_handler(p_ble_lls_c, &evt);
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

        evt.evt_type = BLE_LLS_C_EVT_DISCOVERY_COMPLETE;//BLE_LLS_C_EVT_DISCOVERY_COMPLETE;

        mp_ble_lls_c[lls_c_nbr]->evt_handler(mp_ble_lls_c[lls_c_nbr], &evt);
    }
		else
		{
        ble_lls_c_evt_t evt;
				evt.evt_type = BLE_LLS_C_EVT_SERVICE_NOT_FOUND;//BLE_LLS_C_EVT_DISCOVERY_COMPLETE;
        mp_ble_lls_c[lls_c_nbr]->evt_handler(mp_ble_lls_c[lls_c_nbr], &evt);
		}

}


/***********************************************************************************
															PUBLIC API
************************************************************************************/

uint32_t ble_lls_c_init(ble_lls_c_t **pp_ble_lls_c, uint8_t ble_lls_c_size, ble_lls_c_init_t * p_ble_lls_c_init)
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
        mp_ble_lls_c[i]->evt_handler  = p_ble_lls_c_init->evt_handler;
        mp_ble_lls_c[i]->conn_handle     = BLE_CONN_HANDLE_INVALID;
        mp_ble_lls_c[i]->lls_cccd_handle = BLE_GATT_HANDLE_INVALID;
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

	
	ble_gatts_evt_t gatt_server = p_ble_evt->evt.gatts_evt;
	ble_gattc_evt_write_rsp_t gatt_client = p_ble_evt->evt.gattc_evt.params.write_rsp;
	
    if ((p_ble_lls_c == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            p_ble_lls_c->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
				    printf("Connected !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n\r\n");
            break;
				
				case BLE_GATTC_EVT_CHAR_DISC_RSP:
					
						//ble_lls_c_alert_set( p_ble_lls_c);
						break;
				case BLE_GATTC_EVT_WRITE_RSP:
					printf("GATT client: %x %x %x %x %x. CODE is %x\r\n", *gatt_client.data, gatt_client.handle, gatt_client.len, 
											gatt_client.offset, gatt_client.write_op, p_ble_evt->evt.gattc_evt.gatt_status);
				nrf_delay_ms(5000);
											ble_lls_alert_get(p_ble_lls_c);
				break;
				case BLE_GATTC_EVT_DESC_DISC_RSP:
					LOG("2.WRITE value is %x\r\n",* p_ble_evt->evt.gattc_evt.params.write_rsp.data);
											  ble_lls_alert_get(p_ble_lls_c);
									break;
				case BLE_GATTC_EVT_READ_RSP:
				  LOG("READ value is %x\r\n", *p_ble_evt->evt.gattc_evt.params.read_rsp.data);
						break;
				case BLE_EVT_TX_COMPLETE:
						LOG("WRITE value is %x number of packets is %x\r\n", *p_ble_evt->evt.gattc_evt.params.write_rsp.data,
									p_ble_evt->evt.gattc_evt.gatt_status);
				//  ble_lls_alert_get(p_ble_lls_c);


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
		
		 alert_value=BLE_CHAR_ALERT_LEVEL_HIGH_ALERT;
		// ble_lls_c_service_set_value(p_ble_lls_c);
		
		 LOG("[LLS_C]: Saving a alert level as High Alert in the server characteristic.\r\n");
		
		 return NRF_SUCCESS;
}

 uint32_t ble_lls_c_alert_remove(ble_lls_c_t * p_ble_lls_c)
{
    if (p_ble_lls_c == NULL)
    {
        return NRF_ERROR_NULL;
    }	
		
		 alert_value=BLE_CHAR_ALERT_LEVEL_HIGH_ALERT;
//		 ble_lls_c_service_set_value(p_ble_lls_c);
		
		 LOG("[LLS_C]: Saving a alert level as No Alert in the server characteristic.\r\n");
		
		 return NRF_SUCCESS;
}

void ble_lls_alert_get(ble_lls_c_t * p_lls)
{
		sd_ble_gattc_read(p_lls->conn_handle, p_lls->llm_value_handle,0);
}


uint32_t ble_lls_c_llm_notif_enable(ble_lls_c_t * p_ble_lls_c)
{
    if (p_ble_lls_c == NULL)
    {
        return NRF_ERROR_NULL;
    }

    return cccd_configure(p_ble_lls_c->conn_handle, p_ble_lls_c->lls_cccd_handle, true);
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
 uint32_t write_characteristic_value(uint16_t  conn_handle,
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

