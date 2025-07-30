#include <stdio.h>
#include "pico/stdlib.h"
#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "btstack_event.h"

#define SERVER_DEVICE_NAME "PicoW-HT"
typedef enum {
    CLIENT_STOP=0,
    QUERY_SERVICE,
    QUERY_CHARACTERISTIC_TEMPERATURE,
    QUERY_CHARACTERISTIC_HUMIDITY,
    ENABLE_NOTIFICATION,
    ENABLE_COMPLETE
} client_event_query_t;

float rtemp, rhumi, ntemp, nhumi;

client_event_query_t client_event_query;

gatt_client_notification_t notification_listener_temp, notification_listener_humi;
static hci_con_handle_t connection_handler;
gatt_client_service_t environmental_sensing;
gatt_client_characteristic_t temperature, humidity;
static btstack_packet_callback_registration_t hci_event_callback_registration;

static void handle_hci_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void handle_gatt_client_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

uint8_t  advertisement_report_find_device(uint16_t uuid16, const char * name, uint8_t * advertisement_report){
    // get advertisement from report event
    const uint8_t * adv_data = gap_event_advertising_report_get_data(advertisement_report);
    uint8_t         adv_len  = gap_event_advertising_report_get_data_length(advertisement_report);

    // iterate over advertisement data
    ad_context_t context;

    char local_name[40];
    uint8_t found = 0;
    for (ad_iterator_init(&context, adv_len, adv_data) ; ad_iterator_has_more(&context) ; ad_iterator_next(&context)){
        uint8_t data_type    = ad_iterator_get_data_type(&context);
        uint8_t data_size    = ad_iterator_get_data_len(&context);
        const uint8_t * data = ad_iterator_get_data(&context);
        int i;
        switch (data_type){
            case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS:
                for (i=0; i<data_size;i+=2){
                    if (little_endian_read_16(data, i) == uuid16) {
                        found = found | 0x1;
                        break;
                    }
                }
                break;
            case BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME:
                memcpy(local_name, data, data_size);
                local_name[data_size]='\0';
                if (strcmp(local_name, name) == 0) {
                    found = found | 0x2;
                }
            break;
            default:
                break;
        }
    }
    return found;
}

static void handle_hci_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);
    bd_addr_t addr;
    bd_addr_type_t addr_type; 
    if (packet_type != HCI_EVENT_PACKET) return;
    
    uint8_t event = hci_event_packet_get_type(packet);
    switch (event) {
        case BTSTACK_EVENT_STATE:
            // BTstack activated, get started
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) break;
            gap_set_scan_parameters(0,0x0030, 0x0030);
            gap_start_scan();
            break;
        case GAP_EVENT_ADVERTISING_REPORT:
            if (advertisement_report_find_device(ORG_BLUETOOTH_SERVICE_ENVIRONMENTAL_SENSING, SERVER_DEVICE_NAME, packet)==0x3) {
                gap_event_advertising_report_get_address(packet, addr);
                addr_type = gap_event_advertising_report_get_address_type(packet);
                gap_stop_scan();
                gap_connect(addr, addr_type);
            }
            break;
        case HCI_EVENT_LE_META:
            // wait for connection complete
            if (hci_event_le_meta_get_subevent_code(packet) !=  HCI_SUBEVENT_LE_CONNECTION_COMPLETE) break;
            connection_handler = hci_subevent_le_connection_complete_get_connection_handle(packet);
            // query primary services
            client_event_query=QUERY_SERVICE;
            gatt_client_discover_primary_services_by_uuid16(handle_gatt_client_event, connection_handler,
                        ORG_BLUETOOTH_SERVICE_ENVIRONMENTAL_SENSING);
            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            client_event_query = CLIENT_STOP;
            gap_start_scan();
            break;
        default:
            break;
    }
}
static void handle_gatt_client_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(packet_type);
    UNUSED(channel);
    UNUSED(size);
    uint16_t temp;

    switch(hci_event_packet_get_type(packet)){
        case GATT_EVENT_SERVICE_QUERY_RESULT:

            gatt_event_service_query_result_get_service(packet, &environmental_sensing);
        break;
        case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT:

            if (client_event_query == QUERY_CHARACTERISTIC_TEMPERATURE) { 

                gatt_event_characteristic_query_result_get_characteristic(packet, &temperature);

            }
            if (client_event_query == QUERY_CHARACTERISTIC_HUMIDITY) { 

                gatt_event_characteristic_query_result_get_characteristic(packet, &humidity);

            }
        break;
        case GATT_EVENT_QUERY_COMPLETE:

            switch(client_event_query) {
                case QUERY_SERVICE:
                    client_event_query = QUERY_CHARACTERISTIC_TEMPERATURE;
                    gatt_client_discover_characteristics_for_service_by_uuid16(handle_gatt_client_event, connection_handler, 
                        &environmental_sensing, ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE);                
                break;
                case QUERY_CHARACTERISTIC_HUMIDITY:
                    client_event_query = ENABLE_NOTIFICATION;
                    gatt_client_listen_for_characteristic_value_updates(&notification_listener_temp, handle_gatt_client_event, 
                        connection_handler, &temperature);
                    gatt_client_listen_for_characteristic_value_updates(&notification_listener_humi, handle_gatt_client_event, 
                       connection_handler, &humidity);
                    // enable notifications
                    gatt_client_write_client_characteristic_configuration(handle_gatt_client_event, connection_handler,
                        &temperature, GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION);
                    //gatt_client_write_client_characteristic_configuration(handle_gatt_client_event, connection_handler,
                     //   &humidity, GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION);

                break;
                case QUERY_CHARACTERISTIC_TEMPERATURE:
                    client_event_query = QUERY_CHARACTERISTIC_HUMIDITY;
                    gatt_client_discover_characteristics_for_service_by_uuid16(handle_gatt_client_event, connection_handler, 
                        &environmental_sensing,ORG_BLUETOOTH_CHARACTERISTIC_HUMIDITY);
                break;
                case ENABLE_NOTIFICATION:
                client_event_query = ENABLE_COMPLETE;
                            
                break;
            }
            break;
            case GATT_EVENT_CHARACTERISTIC_VALUE_QUERY_RESULT:
                if (humidity.value_handle == gatt_event_characteristic_value_query_result_get_value_handle(packet)) { 
                    temp = little_endian_read_16(gatt_event_characteristic_value_query_result_get_value(packet), 0);
                    rhumi = temp/100.0;
                    printf("humidity read:%.02f%c\n", rhumi, '%');
                }
                if (temperature.value_handle == gatt_event_characteristic_value_query_result_get_value_handle(packet)) { 
                    temp = little_endian_read_16(gatt_event_characteristic_value_query_result_get_value(packet), 0);
                    rtemp = temp/100.0;
                    printf("temperature read:%.02f C\n", rtemp);
                }

                 
            break;
            case GATT_EVENT_NOTIFICATION:
                if (temperature.value_handle == gatt_event_notification_get_value_handle(packet)) { 
                    ntemp = little_endian_read_16(gatt_event_notification_get_value(packet), 0)/100.0;
                    printf("temp:%.02f C\n", ntemp); 
                }
                if (humidity.value_handle == gatt_event_notification_get_value_handle(packet)) { 
                    nhumi = little_endian_read_16(gatt_event_notification_get_value(packet), 0)/100.0;
                    printf("humi:%.02f %c\n", nhumi, '%');
                }
            break;
            case ATT_EVENT_CAN_SEND_NOW:
            printf("att send\n");
            break;
        default:
            break;
    }
}


int main()
{
    stdio_init_all();
    
    if (cyw43_arch_init()) {
        printf("cyw43_init error\n");
        return 0;
    }
    client_event_query = CLIENT_STOP;
    
    l2cap_init();
    sm_init();

      // setup GATT client
    l2cap_init();

    // Initialize GATT client 
    gatt_client_init();

    // Optinoally, Setup security manager
    sm_init();
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);

    // register for HCI events
    hci_event_callback_registration.callback = &handle_hci_event;
    hci_add_event_handler(&hci_event_callback_registration);


    hci_power_control(HCI_POWER_ON);
    while(1) {
        if (client_event_query == ENABLE_COMPLETE) {
            sleep_ms(4500);
            gatt_client_read_value_of_characteristics_by_uuid16(handle_gatt_client_event, connection_handler, temperature.start_handle, temperature.end_handle, ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE);
            sleep_ms(100);
            gatt_client_read_value_of_characteristics_by_uuid16(handle_gatt_client_event, connection_handler, humidity.start_handle, humidity.end_handle, ORG_BLUETOOTH_CHARACTERISTIC_HUMIDITY);
        }
        tight_loop_contents();
    }
    

    return 0;
}
