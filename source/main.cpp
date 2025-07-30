#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "btstack.h"
#include "btstack_event.h"

static btstack_packet_callback_registration_t hci_event_callback_registration;

const static uint8_t RSSI_MAX = 190;

static hci_con_handle_t connectionHandle;
static bd_addr_t        address;
static bd_addr_type_t   addressType;
static int              listenerRegistered;
static gatt_client_notification_t notificationListener;

static gatt_client_service_t        heartRateService;
static gatt_client_characteristic_t bodySensorLocationCharacteristic;
static gatt_client_characteristic_t heartRateMeasurementCharacteristic;

typedef enum {
    IDLE,
    SCAN_RESULT,
    CONNECT,
    SERVICE_RESULT,
    HEART_RATE_MEASUREMENT_CHARACTERISTIC,
    ENABLE_NOTIFICATIONS_COMPLETE,
    SENSOR_LOCATION_CHARACTERISTIC,
    SENSOR_LOCATION,
    CONNECTED
} state_t;

static state_t state = IDLE;

/* @section GATT client setup
 *
 * @text In the setup phase, a GATT client must register the HCI and GATT client
 * packet handlers, as shown in Listing GATTClientSetup.
 * Additionally, the security manager can be setup, if signed writes, or
 * encrypted, or authenticated connection are required, to access the
 * characteristics, as explained in Section on [SMP](../protocols/#sec:smpProtocols).
 */

/* LISTING_START(GATTClientSetup): Setting up GATT client */

// Handles connect, disconnect, and advertising report events,  
// starts the GATT client, and sends the first query.
static void handle_hci_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

// Handles GATT client query results, sends queries and the 
// GAP disconnect command when the querying is done.
static void handle_gatt_client_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static uint8_t isItCyclingSensor(uint8_t *report, uint16_t uuid16)
{
    const uint8_t * adv_data = gap_event_advertising_report_get_data(report);
    uint8_t         adv_len  = gap_event_advertising_report_get_data_length(report);

    // iterate over advertisement data
    ad_context_t context;
    uint8_t found = 0;
    uint8_t rssi;
    bd_addr_t address;
    memset(address, 0, sizeof(bd_addr_t));
    for (ad_iterator_init(&context, adv_len, adv_data) ; ad_iterator_has_more(&context) ; ad_iterator_next(&context)){
        uint8_t data_type    = ad_iterator_get_data_type(&context);
	uint8_t data_size    = ad_iterator_get_data_len(&context);
        const uint8_t * data = ad_iterator_get_data(&context);
        switch (data_type){
	case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS:
	case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS:
	    // compare common prefix
	    for (int i=0; i<data_size;i+=2){
		rssi = gap_event_advertising_report_get_rssi(report);
		if(little_endian_read_16(data, i) == uuid16 && rssi >= RSSI_MAX){
		    gap_event_advertising_report_get_address(report, address);
		    printf("found HR monitor -- address: %s address type: 0x%02X\n", bd_addr_to_str(address), gap_event_advertising_report_get_address_type(report));
		    found = 1;
		}
	    }
	    break;
	default:
	    break;
        }

    }
    return found;
}

/* @section HCI packet handler
 * 
 * @text The HCI packet handler has to start the scanning, 
 * to find the first advertising device, to stop scanning, to connect
 * to and later to disconnect from it, to start the GATT client upon
 * the connection is completed, and to send the first query - in this
 * case the gatt_client_discover_primary_services() is called, see 
 * Listing GATTBrowserHCIPacketHandler.  
 */

/* LISTING_START(GATTBrowserHCIPacketHandler): Connecting and disconnecting from the GATT client */
static void handle_hci_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) return;
    uint16_t connInterval;
    bd_addr_t conAddr;
    uint8_t event = hci_event_packet_get_type(packet);
    switch (event) {
        case BTSTACK_EVENT_STATE:
            // BTstack activated, get started
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) break;
	    //TODO if device is paired try to connect to it else scan
            printf("BTstack activated, start scanning!\n");
            gap_set_scan_parameters(0,0x0030, 0x0030);
            gap_start_scan();
            break;
        case GAP_EVENT_ADVERTISING_REPORT:
	    if (!isItCyclingSensor(packet, ORG_BLUETOOTH_SERVICE_HEART_RATE)) return;
	    gap_event_advertising_report_get_address(packet, address);
            addressType = static_cast<bd_addr_type_t>(gap_event_advertising_report_get_address_type(packet));
            // stop scanning, and connect to the device
            gap_stop_scan();
	    printf("Stop scan. Connect to device with addr %s and addr type: %d.\n", bd_addr_to_str(address), addressType);
            gap_connect(address,static_cast<bd_addr_type_t>(addressType)); 
            break;
        case HCI_EVENT_META_GAP:
            // wait for connection complete
            if (hci_event_gap_meta_get_subevent_code(packet) !=  GAP_SUBEVENT_LE_CONNECTION_COMPLETE) break;
            printf("\nGATT browser - CONNECTED\n");
            connectionHandle = gap_subevent_le_connection_complete_get_connection_handle(packet);
	    gap_subevent_le_connection_complete_get_peer_address(packet, conAddr);
	    printf("connected to %s on handle: 0x%04X\n", bd_addr_to_str(conAddr), connectionHandle);
	    // print connection parameters (without using float operations)
	    connInterval = gap_subevent_le_connection_complete_get_conn_interval(packet);
            printf("Connection Interval: %u.%02u ms\n", connInterval * 125 / 100, 25 * (connInterval & 3));
            printf("Connection Latency: %u\n", gap_subevent_le_connection_complete_get_conn_latency(packet));
            // initialize gatt client context with handle, and add it to the list of active clients
            // query primary services
            printf("Search for Heart Rate service.\n");
            // query primary services
            // gatt_client_discover_primary_services(handle_gatt_client_event, connectionHandle);
	    state = SERVICE_RESULT;
            gatt_client_discover_primary_services_by_uuid16(handle_gatt_client_event, connectionHandle, ORG_BLUETOOTH_SERVICE_HEART_RATE);
            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE:
	    connectionHandle = HCI_CON_HANDLE_INVALID;
            if (listenerRegistered){
                listenerRegistered = 0;
                gatt_client_stop_listening_for_characteristic_value_updates(&notificationListener);
            }

            printf("Disconnected %s\n", bd_addr_to_str(address));
	    
            printf("\nGATT browser - DISCONNECTED handle: 0x%04X\n", hci_event_disconnection_complete_get_connection_handle(packet));
            break;
        default:
            break;
    }
}
/* LISTING_END */

/* @section GATT Client event handler
 *
 * @text Query results and further queries are handled by the GATT client packet
 * handler, as shown in Listing GATTBrowserQueryHandler. Here, upon
 * receiving the primary services, the
 * gatt_client_discover_characteristics_for_service() query for the last
 * received service is sent. After receiving the characteristics for the service,
 * gap_disconnect is called to terminate the connection. Upon
 * disconnect, the HCI packet handler receives the disconnect complete event.  
 */

/* LISTING_START(GATTBrowserQueryHandler): Handling of the GATT client queries */

static void handle_gatt_client_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(packet_type);
    UNUSED(channel);
    UNUSED(size);

    uint16_t heart_rate;
    uint8_t  sensor_contact;
    uint8_t  att_status;

    switch(state){
    case SERVICE_RESULT:
	switch(hci_event_packet_get_type(packet)){
	case GATT_EVENT_SERVICE_QUERY_RESULT:
	    // store service (we expect only one)
	    gatt_event_service_query_result_get_service(packet, &heartRateService);
	    break;
	case GATT_EVENT_QUERY_COMPLETE:
	    att_status = gatt_event_query_complete_get_att_status(packet);
	    if (att_status != ATT_ERROR_SUCCESS){
		printf("SERVICE_QUERY_RESULT - ATT Error 0x%02x.\n", att_status);
		gap_disconnect(connectionHandle);
		break;  
	    } 
	    state = HEART_RATE_MEASUREMENT_CHARACTERISTIC;
	    printf("Search for Heart Rate Measurement characteristic.\n");
	    gatt_client_discover_characteristics_for_service_by_uuid16(handle_gatt_client_event, connectionHandle, &heartRateService, ORG_BLUETOOTH_CHARACTERISTIC_HEART_RATE_MEASUREMENT);
	    break;
	default:
	    break;
	}
	break;
    case HEART_RATE_MEASUREMENT_CHARACTERISTIC:
	switch(hci_event_packet_get_type(packet)){
	case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT:
	    gatt_event_characteristic_query_result_get_characteristic(packet, &heartRateMeasurementCharacteristic);
	    break;
	case GATT_EVENT_QUERY_COMPLETE:
	    att_status = gatt_event_query_complete_get_att_status(packet);
	    if (att_status != ATT_ERROR_SUCCESS){
		printf("CHARACTERISTIC_QUERY_RESULT - ATT Error 0x%02x.\n", att_status);
		gap_disconnect(connectionHandle);
		break;  
	    } 
	    // register handler for notifications
	    listenerRegistered = 1;
	    gatt_client_listen_for_characteristic_value_updates(&notificationListener, handle_gatt_client_event, connectionHandle, &heartRateMeasurementCharacteristic);
	    // enable notifications
	    printf("Enable Notify on Heart Rate Measurements characteristic.\n");
	    state = CONNECTED;
	    gatt_client_write_client_characteristic_configuration(handle_gatt_client_event, connectionHandle,
								  &heartRateMeasurementCharacteristic, GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION);
	    break;
	default:
	    break;
	}
	break;
    case CONNECTED:
	switch(hci_event_packet_get_type(packet)){
	case GATT_EVENT_NOTIFICATION:
	    if (gatt_event_notification_get_value(packet)[0] & 1){
		heart_rate = little_endian_read_16(gatt_event_notification_get_value(packet), 1);
	    } else {
		heart_rate = gatt_event_notification_get_value(packet)[1];
	    }
	    sensor_contact = (gatt_event_notification_get_value(packet)[0] >> 1) & 3;
	    printf("Heart Rate: %3u\n", heart_rate);
	    break;
	case GATT_EVENT_QUERY_COMPLETE:
	    break;
	case GATT_EVENT_CAN_WRITE_WITHOUT_RESPONSE:
	    break;
	default:
	    break;
	}
	break;
    default:
	break;
    }

}
/* LISTING_END */


int btInit(void) {
    // initialize CYW43 driver architecture (will enable BT if/because CYW43_ENABLE_BLUETOOTH == 1)
    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        return -1;
    }
    
    printf("%s\n", __func__);

    return 0;
}

void led_task(void* pvParameters)
{   
    const uint LED_PIN = CYW43_WL_GPIO_LED_PIN;
    
    cyw43_arch_gpio_put(LED_PIN, 0);
    while (true) {
        cyw43_arch_gpio_put(LED_PIN, 1);
	printf("LED On!\n");
        vTaskDelay(500);
        cyw43_arch_gpio_put(LED_PIN, 0);
	printf("LED Off!\n");
        vTaskDelay(500);
    }
}

void bt_task(void *pvParameters)
{
    int ret = btInit();
    if(ret){
	printf("failed to init\n");
	return;
    }
    else{
	printf("initialized\n");
    }

    l2cap_init();

    gatt_client_init();

    sm_init();
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);

    hci_event_callback_registration.callback = &handle_hci_event;
    hci_add_event_handler(&hci_event_callback_registration);

    // turn on!
    hci_power_control(HCI_POWER_ON);
    
    while(true){
	vTaskDelay(100);
    }
}

int main()
{
    stdio_init_all();

    // xTaskCreate(led_task, "LED_Task", 256, NULL, 1, NULL);
    xTaskCreate(bt_task, "BT_Task", 256, NULL, 1, NULL);
    vTaskStartScheduler();

    while(1){};
}
