/*
 Name:		WiFiCon_ESP32.ino
 Created:	2/10/2021 4:36:16 PM
 Author:	Brandon Van Pelt
 Description: This program is a CAN Bus to WiFi bridge
 Current use: CAN Bus -> ESP32 -> WifI -> ESP-12E - > Arduino Due Controller
*/

// This device MAC Address:  24:6F:28:9D:A7:8C

#if CONFIG_FREERTOS_UNICORE

#define ESP32_RUNNING_CORE 0
#else
#define ESP32_RUNNING_CORE 1
#endif

//#include <can_common.h>
#include <esp32_can.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>

//#define DEBUG
#define CAN_BAUD_RATE 500000
#define SERIAL_BAUD_RATE 115200
#define ID_MATCH 0xFFF
#define ARM1_RX 0x0C1
#define ARM2_RX 0x0C2

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = { 0xD8, 0xF1, 0x5B, 0x15, 0x8E, 0x9A };

// Task declarations
void TaskEmptyBuffer(void* pvParameters);

// CAN Bus structure
typedef struct struct_message {
    uint16_t ID;
    uint8_t MSG[8];
} struct_message;

struct_message rxCANFrame[20];
struct_message txCANFrame;

void printFrame(CAN_FRAME* message)
{
    Serial.print(message->id, HEX);
    if (message->extended) Serial.print(" X ");
    else Serial.print(" S ");
    Serial.print(message->length, DEC);
    for (int i = 0; i < message->length; i++) {
        Serial.print(message->data.byte[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

/*=========================================================
            Circular Buffer
===========================================================*/
uint8_t CANInPtr = 0;
uint8_t CANOutPtr = 0;
extern uint8_t CAN_MSG_In_Buff = 0;

uint8_t CAN_Buff_In()
{
    uint8_t temp = CANInPtr;
    if (CANInPtr < 18)
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
        if (CANOutPtr < 18)
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


/*=========================================================
            Callbacks
===========================================================*/
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len)
{
#if defined DEBUG
    Serial.println("recieved");
#endif
    
    uint8_t buffPtr = CAN_Buff_In();
    memcpy(&rxCANFrame[buffPtr], incomingData, sizeof(rxCANFrame[buffPtr]));

#if defined DEBUG
    Serial.println(CAN_MSG_In_Buff);
#endif
}

void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) 
{
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void CANBusRX(CAN_FRAME* frame)
{
    printFrame(frame);
    txCANFrame.ID = frame->id;
    for (uint8_t i = 0; i < 8; i++)
    {
        txCANFrame.MSG[i] = frame->data.uint8[i];
    }
    esp_now_send(broadcastAddress, (uint8_t*)&txCANFrame, sizeof(txCANFrame));
}


/*=========================================================
            Tasks
===========================================================*/
void TaskEmptyBuffer(void* pvParameters)
{
    (void)pvParameters;
    for (;;) 
    {
#if defined DEBUG
        Serial.println(CAN_MSG_In_Buff);
#endif

        if (CAN_MSG_In_Buff > 0)
        {
            // Get CAN Bus Frame buffer location
            uint8_t buffPtr = CAN_Buff_Out();

            CAN_FRAME TxFrame;
            TxFrame.rtr = 0;
            TxFrame.id = rxCANFrame[buffPtr].ID;
            TxFrame.extended = false;
            TxFrame.length = 8;
            for (uint8_t i = 0; i < 8; i++)
            {
                TxFrame.data.uint8[i] = rxCANFrame[buffPtr].MSG[i];
            }
            CAN0.sendFrame(TxFrame);
        }
        vTaskDelay(20);
    }
}


/*=========================================================
            Setup
===========================================================*/
void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register peer
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println(F("Failed to add peer"));
        return;
    }

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    // CAN Bus setup
    CAN0.setCANPins(GPIO_NUM_26, GPIO_NUM_25);
    CAN0.begin(CAN_BAUD_RATE);
    CAN0.watchFor(ARM1_RX, ID_MATCH); // Filter 0
    CAN0.setCallback(0, CANBusRX); // Callback filter 0
    CAN0.watchFor(ARM2_RX, ID_MATCH); // Filter 1
    CAN0.setCallback(1, CANBusRX); // Callback filter 1
    Serial.println(F("Setup complete"));

    xTaskCreatePinnedToCore(
        TaskEmptyBuffer
        , "TaskEmptyBuffer"
        , 1024  // This stack size can be checked & adjusted by reading the Stack Highwater
        , NULL
        , 1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        , NULL
        , ESP32_RUNNING_CORE);
}

void loop()
{
    // Nothing to see here
}
