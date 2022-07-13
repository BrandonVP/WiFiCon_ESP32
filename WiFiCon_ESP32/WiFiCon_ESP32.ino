/*
 Name:		WiFiCon_ESP32.ino
 Created:	2/10/2021 4:36:16 PM
 Author:	Brandon Van Pelt
 Description: This program is a CAN Bus to WiFi bridge
 Current uses: Mega2560 -> ~CAN Bus~ -> ESP32 -> ~WifI~ -> ESP-12E -> ~Serial~ -> Arduino Due Controller
               Wireless CAN Bus streaming for ScanTool project
*/

#include <esp32_can.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include <esp_wifi.h>
#include "RGB_LED.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#if CONFIG_FREERTOS_UNICORE
#define ESP32_RUNNING_CORE 0
#else
#define ESP32_RUNNING_CORE 1
#endif

//#define DEBUG_OnDataRecv
//#define DEBUG_TaskEmptyBuffer
//#define DEBUG_OnDataSent
#define DEBUG_CANBusRX
#define DEBUG_CANBusTX
#define DEBUG_TURN_OFF_ESPNOW // Causing a core panic during testing when called and not connected

#define CAN_BAUD_RATE 500000
#define SERIAL_BAUD_RATE 115200
#define ID_MATCH 0xFFF
#define CAN_BUFFER_SIZE 32

//#define SIXDOF
#if defined SIXDOF
#define ARM1_RX 0x0C1
#define ARM2_RX 0x0C2
#endif

/*
* LED Strobe Assignments
* None: RED
* Wifi RX: PURPLE
* WiFi TX: TEAL
* Can Bus RX: GREEN
* Can Bus TX: BLUE
*/

/*
Dongle Beta Unit Mac Addresses
1: C8:C9:A3:F9:FD:04
2: 
3: 
4:
5: 

V1.2: C8:C9:A3:FB:20:20
*/

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = { 0xD8, 0xF1, 0x5B, 0x15, 0x8E, 0x9A }; // 6DOF Remote
//uint8_t broadcastAddress[] = { 0x9C, 0x9C, 0x1F, 0xDD, 0x4B, 0xD0 }; // ScanTool B
//uint8_t broadcastAddress[] = { 0x9C, 0x9C, 0x1F, 0xDD, 0x4B, 0xD0 }; // ScanTool H

// Task declarations
void TaskEmptyBuffer(void* pvParameters);

// Semaphore handle for shared LED resources
SemaphoreHandle_t xLEDSemaphore;

// CAN Bus structure
typedef struct struct_message 
{
    uint16_t ID;
    uint8_t MSG[8];
} struct_message;

static struct_message rxCANFrame[CAN_BUFFER_SIZE];
static struct_message txCANFrame;

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
static uint8_t bufferInPtr = 0;
static uint8_t bufferOutPtr = 0;

uint8_t CAN_Buff_In()
{
    uint8_t temp = bufferInPtr;
    (bufferInPtr < CAN_BUFFER_SIZE - 1) ? bufferInPtr++ : bufferInPtr = 0;
    if (bufferInPtr == bufferOutPtr)
    {
        (bufferOutPtr < (CAN_BUFFER_SIZE - 1)) ? bufferOutPtr++ : bufferOutPtr = 0;
    }
    return temp;
}

uint8_t CAN_Buff_Out()
{
    uint8_t temp = bufferOutPtr;
    (bufferOutPtr < CAN_BUFFER_SIZE - 1) ? bufferOutPtr++ : bufferOutPtr = 0;
    return temp;
}

// Calculates current structures in buffer by subtracting pointers then anding with max buffer size value
uint8_t stack_size()
{
    uint8_t size = (bufferInPtr - bufferOutPtr) & (CAN_BUFFER_SIZE - 1);
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

    // Add strobe to que
    xSemaphoreTake(xLEDSemaphore, portMAX_DELAY);
    strobeQue((RGB)PURPLE);
    xSemaphoreGive(xLEDSemaphore);

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
    Serial.print(F("\r\nLast Packet Send Status:\t"));
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
#endif

    // Add strobe to que
    xSemaphoreTake(xLEDSemaphore, portMAX_DELAY);
    strobeQue((RGB)TEAL);
    xSemaphoreGive(xLEDSemaphore);
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

    // TODO: Can the connection be checked before calling?
#if !defined DEBUG_TURN_OFF_ESPNOW
        esp_now_send(broadcastAddress, (uint8_t*)&txCANFrame, sizeof(txCANFrame));
#endif
    
    // Add strobe to que
    xSemaphoreTake(xLEDSemaphore, portMAX_DELAY);
    strobeQue((RGB)GREEN);
    xSemaphoreGive(xLEDSemaphore); 
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
            CAN0.sendFrame(TxFrame);

            // Add strobe to que
            xSemaphoreTake(xLEDSemaphore, portMAX_DELAY);
            strobeQue((RGB)BLUE);
            xSemaphoreGive(xLEDSemaphore);

#if defined DEBUG_TaskEmptyBuffer
            printFrame(&TxFrame);
#endif
        }
        // TODO: Testing needed to determine delay
        vTaskDelay(2);
    }
}

// Task for RGB LED control
void TaskLEDControl(void* pvParameters)
{
    (void)pvParameters;
    for (;;)
    {
        // Traffic status LED
        xSemaphoreTake(xLEDSemaphore, portMAX_DELAY);
        strobe_LED((RGB)RED);
        xSemaphoreGive(xLEDSemaphore);

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
    Serial.print(F("Mac Address: "));
    Serial.print(WiFi.macAddress());

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        Serial.println(F("Error initializing ESP-NOW"));
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
    xLEDSemaphore = xSemaphoreCreateCounting(1, 1);
    xSemaphoreGive(xLEDSemaphore);
    xTaskCreatePinnedToCore(TaskEmptyBuffer, "TaskEmptyBuffer", 1024  , NULL, 2, NULL, ESP32_RUNNING_CORE);
    xTaskCreatePinnedToCore(TaskLEDControl, "TaskLEDControl", 1024, NULL, 3, NULL, ESP32_RUNNING_CORE);
    vTaskStartScheduler();
}

// For testing new hardware
void CANBusTXTest()
{
    static uint32_t timer55 = 0;
    if (millis() - timer55 > 2000)
    {
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

        // Add strobe to que
        xSemaphoreTake(xLEDSemaphore, portMAX_DELAY);
        strobeQue(BLUE);
        xSemaphoreGive(xLEDSemaphore);

        Serial.println(F("Message sent"));
        timer55 = millis();
    }
}

// All functions are event driven
void loop(){}
