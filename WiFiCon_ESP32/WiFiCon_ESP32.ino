/*
 Name:		WiFiCon_ESP32.ino
 Created:	2/10/2021 4:36:16 PM
 Author:	Brandon Van Pelt
*/

#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>

// CAN Bus Variables
CAN_device_t CAN_cfg;               // CAN Config
unsigned long previousMillis = 0;   // will store last time a CAN Message was send
const int interval = 1000;          // interval at which send CAN Messages (milliseconds)
const int rx_queue_size = 10;       // Receive Queue size
CAN_frame_t rx_frame;

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = { 0xD8, 0xF1, 0x5B, 0x15, 0x8E, 0x9A };

// Variable to store if sending data was successful
String success;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    uint16_t ID;
    uint8_t MSG[8];
} struct_message;

struct_message rxCANFrame[10];
struct_message txCANFrame;

uint8_t CANInPtr = 0;
uint8_t CANOutPtr = 0;
uint8_t CAN_MSG_In_Buff = 0;

uint8_t CAN_Buff_In()
{
    uint8_t temp = CANInPtr;
    if (CANInPtr < 8)
    {
        CANInPtr++;
    }
    else
    {
        CANInPtr = 0;
    }
    CAN_MSG_In_Buff++;
    return temp;
}

uint8_t CAN_Buff_Out()
{
    if (CAN_MSG_In_Buff > 0)
    {
        uint8_t temp = CANOutPtr;
        if (CANOutPtr < 8)
        {
            CANOutPtr++;
        }
        else
        {
            CANOutPtr = 0;
        }
        CAN_MSG_In_Buff--;
        return temp;
    }
    return 0;
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len)
{
    uint8_t buffPtr = CAN_Buff_In();
    memcpy(&rxCANFrame[buffPtr], incomingData, sizeof(rxCANFrame[buffPtr]));
}

// Callback when data is sent
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    if (status == 0) {
        success = "Delivery Success :)";
    }
    else {
        success = "Delivery Fail :(";
    }
}

void setup()
{
    // Initialize Serial Monitor
    Serial.begin(115200);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);

    // Register peer
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(OnDataRecv);

    // CAN Bus setup
    CAN_cfg.speed = CAN_SPEED_500KBPS;
    CAN_cfg.tx_pin_id = GPIO_NUM_25;
    CAN_cfg.rx_pin_id = GPIO_NUM_26;
    CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
    // Init CAN Module
    ESP32Can.CANInit();
    Serial.println("Setup Done");
}

void loop()
{
    if (CAN_MSG_In_Buff > 0)
    {
        // Get CAN Bus Frame buffer location
        uint8_t buffPtr = CAN_Buff_Out();

        CAN_frame_t tx_frame;
        tx_frame.FIR.B.FF = CAN_frame_std;
        tx_frame.MsgID = rxCANFrame[buffPtr].ID;
        tx_frame.FIR.B.DLC = 8;
        for (uint8_t i = 0; i < 8; i++)
        {
            tx_frame.data.u8[i] = rxCANFrame[buffPtr].MSG[i];
        }
        ESP32Can.CANWriteFrame(&tx_frame);
    }

    // Receive next CAN frame from queue
    if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
    {
        // Filter messages
        if (rx_frame.MsgID == 0xC1 || rx_frame.MsgID == 0xC2)
        {
            txCANFrame.ID = rx_frame.MsgID;
            for (uint8_t i = 0; i < 8; i++)
            {
                txCANFrame.MSG[i] = rx_frame.data.u8[i];
            }
            esp_now_send(broadcastAddress, (uint8_t*)&txCANFrame, sizeof(txCANFrame));
        }
    }
}