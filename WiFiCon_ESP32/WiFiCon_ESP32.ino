/*
 Name:		WiFiCon_ESP32.ino
 Created:	2/10/2021 4:36:16 PM
 Author:	Brandon Van Pelt
 Description: This program is a CAN Bus to WiFi bridge
 Current use: Mega2560 -> ~CAN Bus~ -> ESP32 -> ~WifI~ -> ESP-12E -> ~Serial~ -> Arduino Due Controller
*/
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

//#define DEBUG_OnDataRecv
//#define DEBUG_TaskEmptyBuffer
//#define DEBUG_OnDataSent
//#define DEBUG_CANBusRX
//#define DEBUG_CANBusTX

#if CONFIG_FREERTOS_UNICORE
#define ESP32_RUNNING_CORE 0
#else
#define ESP32_RUNNING_CORE 1
#endif

#include <esp32_can.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include <esp_wifi.h>
#include "RGB_LED.h"

//#define DEBUG
//#define CAN_BAUD_RATE 1000000
#define CAN_BAUD_RATE 500000
#define SERIAL_BAUD_RATE 115200
#define ID_MATCH 0xFFF
#define ARM1_RX 0x0C1
#define ARM2_RX 0x0C2
#define BUFFER_SIZE 32
//#define SIXDOF

// 94:B9:7E:D5:F1:94 // SOC PCB
// Mac Address: C8:C9:A3:FB:20:20 // OBD2 V1.2
// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = { 0xD8, 0xF1, 0x5B, 0x15, 0x8E, 0x9A };
//uint8_t broadcastAddress[] = { 0x9C, 0x9C, 0x1F, 0xDD, 0x4B, 0xD0 };

// Task declarations
void TaskEmptyBuffer(void* pvParameters);

// Create semaphore handle
SemaphoreHandle_t xBufferSemaphore;

// CAN Bus structure
typedef struct struct_message 
{
    uint16_t ID;
    uint8_t MSG[8];
} struct_message;

struct_message rxCANFrame[BUFFER_SIZE];
struct_message txCANFrame;

// esp32_can library print frame method used for testing and debugging
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
uint8_t bufferInPtr = 0;
uint8_t bufferOutPtr = 0;

uint8_t CAN_Buff_In()
{
    uint8_t temp = bufferInPtr;
    (bufferInPtr < BUFFER_SIZE - 1) ? bufferInPtr++ : bufferInPtr = 0;
    if (bufferInPtr == bufferOutPtr)
    {
        (bufferOutPtr < (BUFFER_SIZE - 1)) ? bufferOutPtr++ : bufferOutPtr = 0;
    }
    return temp;
}

uint8_t CAN_Buff_Out()
{
    uint8_t temp = bufferOutPtr;
    (bufferOutPtr < BUFFER_SIZE - 1) ? bufferOutPtr++ : bufferOutPtr = 0;
    return temp;
}

// Calculates current structures in buffer by subtracting pointers then anding with max buffer size value
uint8_t stack_size()
{
    uint8_t size = (bufferInPtr - bufferOutPtr) & (BUFFER_SIZE - 1);
    return size;
}

/*=========================================================
            Callbacks
===========================================================*/
#if defined DEBUG_OnDataRecv
volatile uint16_t test_id = 0;
volatile uint8_t test_data[8];
#endif

// WiFi data received
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len)
{
    uint8_t buffPtr = CAN_Buff_In();
    memcpy(&rxCANFrame[buffPtr], incomingData, sizeof(rxCANFrame[buffPtr]));

    // Add a blue flash to the buffer to indicate incoming WiFi traffic
    xSemaphoreTake(xBufferSemaphore, portMAX_DELAY);
    strobeQue(BLUE);
    xSemaphoreGive(xBufferSemaphore);

#if defined DEBUG_OnDataRecv
    Serial.println("");
    Serial.print("MAC: ");
    Serial.println(mac[0]);
    Serial.print("Len: ");
    Serial.println(len);
    Serial.println("");
    Serial.println("**************OnDataRecv()**************");
    Serial.print("ID: ");
    test_id = rxCANFrame[buffPtr].ID;
    Serial.print(test_id, 16);
    Serial.print(" Ptr: ");
    Serial.println(buffPtr);
#endif
}

// WiFi data sent
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status)
{
#if defined DEBUG_OnDataSent
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
#endif
}

// CAN Bus incoming messages
void CANBusRX(CAN_FRAME* frame)
{
#if defined DEBUG_CANBusRX
    printFrame(frame);
#endif
   
    txCANFrame.ID = frame->id;
    for (uint8_t i = 0; i < 8; i++)
    {
        txCANFrame.MSG[i] = frame->data.uint8[i];
    }
    esp_now_send(broadcastAddress, (uint8_t*)&txCANFrame, sizeof(txCANFrame));

    // Add a green flash to the buffer to indicate incoming CAN Bus traffic
    xSemaphoreTake(xBufferSemaphore, portMAX_DELAY);
    strobeQue(GREEN);
    xSemaphoreGive(xBufferSemaphore);
}

/*=========================================================
            Tasks
===========================================================*/
// Send messages received from WiFi out on the CAN Bus
void TaskEmptyBuffer(void* pvParameters)
{
    (void)pvParameters;
    for (;;)
    {
        // If traffic to empty
        if (stack_size() > 0)
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
#if defined DEBUG_TaskEmptyBuffer
            printFrame(&TxFrame);
#endif
            CAN0.sendFrame(TxFrame);
        }
        vTaskDelay(8);
    }
}

// Task for RGB LED control
void TaskLEDControl(void* pvParameters)
{
    (void)pvParameters;
    for (;;)
    {
        // Traffic status LED
        xSemaphoreTake(xBufferSemaphore, portMAX_DELAY);
        strobe_LED(RED);
        xSemaphoreGive(xBufferSemaphore);

        // Good a place as any to put this tx test line
#if defined DEBUG_CANBusTX
        CANBusTXTest();
#endif
    }
}

/*=========================================================
            Setup
===========================================================*/
void setup()
{
    //disable brownout detector
    //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 

    // For debug
    Serial.begin(SERIAL_BAUD_RATE);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // IF you need the MAC Address
    Serial.print("Mac Address: ");
    Serial.print(WiFi.macAddress());

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
    if (esp_now_add_peer(&peerInfo) != ESP_OK) 
    {
        Serial.println(F("Failed to add peer"));
    }
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    // CAN Bus setup
    CAN0.setCANPins(GPIO_NUM_26, GPIO_NUM_25);
    CAN0.begin(CAN_BAUD_RATE * 2); // Library software bug is halving the baud rate
#if defined SIXDOF
    CAN0.watchFor(ARM1_RX, ID_MATCH); // Filter 0
    CAN0.setCallback(0, CANBusRX); // Callback filter 0
    CAN0.watchFor(ARM2_RX, ID_MATCH); // Filter 1
    CAN0.setCallback(1, CANBusRX); // Callback filter 1
#else
    CAN0.watchFor(0x0, 0x0); // Filter 0
    CAN0.setCallback(0, CANBusRX); // Callback filter 0
#endif
    
    // Setup RGB LEDs
    pinMode(GPIO_NUM_15, OUTPUT);
    pinMode(GPIO_NUM_16, OUTPUT);
    pinMode(GPIO_NUM_17, OUTPUT);
    pinMode(GPIO_NUM_32, OUTPUT);
    pinMode(GPIO_NUM_33, OUTPUT);
    pinMode(GPIO_NUM_14, OUTPUT);

    // CAN BUS Transceiver
    pinMode(GPIO_NUM_27, OUTPUT);
    digitalWrite(GPIO_NUM_27, LOW);

    // Setup FreeRTOS tasks and Semaphore
    xBufferSemaphore = xSemaphoreCreateCounting(1, 1);
    xSemaphoreGive(xBufferSemaphore);
    xTaskCreatePinnedToCore(TaskEmptyBuffer, "TaskEmptyBuffer", 1024  , NULL, 2  , NULL, ESP32_RUNNING_CORE);
    xTaskCreatePinnedToCore(TaskLEDControl, "TaskLEDControl", 1024, NULL, 3, NULL, ESP32_RUNNING_CORE);
    vTaskStartScheduler();
}

// For testing new hardware
void CANBusTXTest()
{
    static uint32_t timer55 = 0;
    if (millis() - timer55 > 1000)
    {
        //digitalWrite(17, !LED);
        CAN_FRAME TxFrame;
        TxFrame.rtr = 0;
        TxFrame.id = 0x01;
        TxFrame.extended = false;
        TxFrame.length = 8;
        for (uint8_t i = 0; i < 8; i++)
        {
            TxFrame.data.uint8[i] = i;
        }
        CAN0.sendFrame(TxFrame);
        Serial.println("sent");
        timer55 = millis();
    }
}


void loop()
{
    // Nothing to see here
}
