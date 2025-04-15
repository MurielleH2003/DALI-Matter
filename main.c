/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#if defined(CONFIG_CLI_SAMPLE_MULTIPROTOCOL)
#include "ble.h"
#endif
#if defined(CONFIG_CLI_SAMPLE_LOW_POWER)
#include "low_power.h"
#endif
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <openthread/dataset_ftd.h>
#include <openthread/config.h>
#include <openthread/cli.h>
#include <openthread/diag.h>
#include <openthread/tasklet.h>
#include <openthread/platform/logging.h>
#include <openthread/instance.h>
#include <openthread/joiner.h>
#include <openthread/thread.h>
#include <openthread/thread_ftd.h>
#include <string.h>
#include <openthread/message.h>
#include <openthread/udp.h>
 //#include <openthread/utils/code_utils.h>
#include <openthread/dataset_ftd.h>
#include <openthread/coap.h>
#include <openthread/message.h>
#include <zephyr/posix/poll.h>
#include <zephyr/posix/sys/socket.h>
#include <unistd.h>
#include <zephyr/posix/netinet/in.h>

/*Includes*/
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <hal/nrf_gpio.h>
#include <stdbool.h>
#include <nrf.h>
#include <nrf52840_peripherals.h>

/*Pindefines*/
#define P1_08 NRF_GPIO_PIN_MAP(1, 8)
#define P1_07 NRF_GPIO_PIN_MAP(1, 7)

/*Taken defines*/
#define STACK_SIZE 1024
#define THREAD_PRIORITY 5

otInstance *instance;
otUdpSocket udpSocket;
otOperationalDataset dataset;
static otCoapResource coapResource;

LOG_MODULE_REGISTER(cli_sample, CONFIG_OT_COMMAND_LINE_INTERFACE_LOG_LEVEL);

K_THREAD_STACK_DEFINE(thread_stack_1, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_stack_2, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_stack_3, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_stack_4, STACK_SIZE);

struct k_thread thread_data_1;
struct k_thread thread_data_2;
struct k_thread thread_data_3;
struct k_thread thread_data_4;

// Semaphore to synchronize task1 and task2
struct k_sem sem_task_sync;
struct k_sem sem_task2_start;


// Declare and initialize the mutex
struct k_mutex my_mutex;

/*Taak intervallen*/
uint32_t task1_interval_us = 1000; // Task 1 interval in microseconds (1 second) 10000 us voor 1*8bits + 1 startbit + 2 stopbits
uint32_t task2_interval_us = 9163;  // Task 2 interval in microseconds (0.5 seconds)
uint32_t task3_interval_us = 1000;  // Task 2 interval in microseconds (0.5 seconds)
uint32_t task4_interval_us = 1000;  // Task 2 interval in microseconds (0.5 seconds)

/*Eigen variabelen*/
typedef enum {
    Idle = 0,
    DetectStartbit1,
    DetectStartbit2,
    ReadBit,
    Delay1,
    Delay2,
    Delay3,
    Stopbit
} verloop;
verloop Verloop = Idle;

uint8_t counter = 0;
uint8_t teller = 0;

uint16_t ontvangen = 0;
static uint16_t verzenden = 0;                 //variabele waar alles in wordt toegevoegd

bool receive = false;
bool sending = true;
bool Data_received = false;        

/*Functies initialiseren*/
uint16_t Tx_data(uint8_t, uint8_t, bool);
void Tx_data_send(uint16_t);
void SysTick_Init(void);
void SysTickDelay(uint32_t);
uint8_t Rx_data_receive(bool);
static void CoapRequestHandler(void *, otMessage *, const otMessageInfo *);
void InitializeCoap(otInstance *);
void StartNet(otInstance*);
void UdpReceiveCallback(void*, otMessage*, const otMessageInfo*);
void StartUDP(otInstance*);
void SendUdpMessage(otInstance*, const char*, const char*, uint16_t);

/*Taken*/
void task1(void *arg1, void *arg2, void *arg3) {
    while (1) {
        // Wait for task2 to signal execution
        k_sem_take(&sem_task_sync, K_FOREVER);
        if (k_mutex_lock(&my_mutex, K_FOREVER) == 0) 
		{
			//printk("Task 1 mutex locked.\n");
            k_usleep(2900);
            ontvangen = Rx_data_receive(receive);
            
			k_mutex_unlock(&my_mutex);
		} 
		else 
		{
			printk("Task 1 failed to lock the mutex.\n");
		}	
        k_sem_reset(&sem_task2_start); // Reset semaphore for task2
        k_sem_reset(&sem_task_sync); // Reset semaphore for task1
        k_usleep(task1_interval_us);
    }
}

void task2(void *arg1, void *arg2, void *arg3) {
    while (1) {
        k_sem_take(&sem_task2_start, K_FOREVER); // Wait for CoAP signal
        
        if (k_mutex_lock(&my_mutex, K_FOREVER) == 0) 
		{
			//printk("Task 3 mutex locked.\n");
             
            if (Data_received == true)
        {
            k_usleep(9163);
        }
        else if (Data_received == false)
        {
            k_usleep(6263);
        }
            
            Tx_data_send(verzenden);                                    //data versturen

			k_mutex_unlock(&my_mutex);
			//printk("Task 2 mutex unlocked.\n");
		} 
		else 
		{
			printk("Task 2 failed to lock the mutex.\n");
		}
        
        k_usleep(task2_interval_us);
    }
}
void DALI_Discover_Devices(void)
{
    for (uint8_t address = 0x00; address <= 0x7F; address++) {
        uint16_t queryCommand = Tx_data(address, 0x90, false); // Query Status command
        sending = true;
        Tx_data_send(queryCommand); // Send command
        k_usleep(20000); // Wait for backward frame
        Rx_data_receive(receive);
        

        if (Data_received) {
            printk("Device found at address: 0x%02x\n", address);
        } else {
            printk("No response from address: 0x%02x\n", address);
        }

        Data_received = false; // Reset flag
    }
    

}

int main(void)
{
	nrf_gpio_cfg_output(P1_08);
    nrf_gpio_cfg_input(P1_07, NRF_GPIO_PIN_NOPULL);

    instance = otInstanceInitSingle();
    memset(&dataset, 0, sizeof(dataset));
	
	k_mutex_init(&my_mutex);
    // Initialize semaphore
    k_sem_init(&sem_task_sync, 0, 1);
    k_sem_init(&sem_task2_start, 0, 1); // Locked initially
    SysTick_Init();

    /*connect to existing network*/

    if (k_mutex_lock(&my_mutex, K_FOREVER) == 0) 
    {
        //printk("Task 3 mutex locked.\n");
         
        StartNet(instance);
        StartUDP(instance);
        InitializeCoap(instance);

        k_mutex_unlock(&my_mutex);
    } 
    else 
    {
        printk("Failed to lock mutex for init.\n");
    }
    otDeviceRole role = otThreadGetDeviceRole(instance);
    printk("Device Role: %d\n", role);
    DALI_Discover_Devices();
    k_tid_t task1_tid = k_thread_create(&thread_data_1, thread_stack_1, STACK_SIZE,
										task1, NULL, NULL, NULL,
										THREAD_PRIORITY, 0, K_NO_WAIT);

	k_tid_t task2_tid = k_thread_create(&thread_data_2, thread_stack_2, STACK_SIZE,
										task2, NULL, NULL, NULL,
										THREAD_PRIORITY, 0, K_NO_WAIT);

   
#if OPENTHREAD_ENABLE_MULTIPLE_INSTANCES
    // Call to query the buffer size
    (void)otInstanceInit(NULL, &otInstanceBufferLength);
    // Call to allocate the buffer
    otInstanceBuffer = (uint8_t*)malloc(otInstanceBufferLength);
    assert(otInstanceBuffer);
    // Initialize OpenThread with the buffer
    instance = otInstanceInit(otInstanceBuffer, &otInstanceBufferLength);
#else
    instance = otInstanceInitSingle();
#endif

#if DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_shell_uart), zephyr_cdc_acm_uart)
    int ret;
    const struct device* dev;
    uint32_t dtr = 0U;
    ret = usb_enable(NULL);
    if (ret != 0 && ret != -EALREADY) {
        LOG_ERR("Failed to enable USB");
        return 0;
    }
    dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));
    if (dev == NULL) {
        LOG_ERR("Failed to find specific UART device");
        return 0;
    }
    LOG_INF("Waiting for host to be ready to communicate");
    /* Data Terminal Ready - check if host is ready to communicate */
    while (!dtr) {
        ret = uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
        if (ret) {
            LOG_ERR("Failed to get Data Terminal Ready line state: %d",
                ret);
            continue;
        }
        k_msleep(100);
    }
    /* Data Carrier Detect Modem - mark connection as established */
    (void)uart_line_ctrl_set(dev, UART_LINE_CTRL_DCD, 1);
    /* Data Set Ready - the NCP SoC is ready to communicate */
    (void)uart_line_ctrl_set(dev, UART_LINE_CTRL_DSR, 1);
#endif
#if defined(CONFIG_CLI_SAMPLE_MULTIPROTOCOL)
    ble_enable();
#endif
#if defined(CONFIG_CLI_SAMPLE_LOW_POWER)
    low_power_enable();
#endif
    return 0;
}

/*Initialisatie SysTick*/
void SysTick_Init(void) 
{
        // SystemCoreClock is typically set to 64 MHz on the nRF52840
        SysTick->LOAD = (SystemCoreClock / 1000000) - 1;  // Load with number of clocks per microsecond
        SysTick->VAL = 0;  // Reset the SysTick counter value
        SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;  // Enable SysTick Timer without interrupt
}

void InitializeCoap(otInstance *instance) 
{
    printk("CoAP opgestart\n");
    memset(&coapResource, 0, sizeof(coapResource));
    coapResource.mUriPath = "nrf52840";
    coapResource.mHandler = CoapRequestHandler;
    coapResource.mContext = instance;

    otError error = otCoapStart(instance, OT_DEFAULT_COAP_PORT);
    if (error == OT_ERROR_NONE) 
	{
        printf("CoAP server started successfully.\n");
    } 
	else 
	{
        printf("Failed to start CoAP server. Error: %d\n", error);
    }

    otCoapAddResource(instance, &coapResource);
    if (error == OT_ERROR_NONE) 
	{
        printf("CoAP resource added successfully.\n");
    } 
	else 
	{
        printf("Failed to add CoAP resource. Error: %d\n", error);
    }
}
// Callback function for receiving UDP messages
void UdpReceiveCallback(void* context, otMessage* message, const otMessageInfo* messageInfo) 
{
    char buffer[128]; // Buffer for the incoming message
    int length = otMessageRead(message, otMessageGetOffset(message), buffer, sizeof(buffer) - 1);

    if (length > 0) 
	{
        buffer[length] = '\0'; // Null-terminate the received message
        printf("Received UDP message: %s\n", buffer);
        verzenden = (uint16_t)atoi(buffer);
        sending = true;
        // Display sender information
        printf("From: %x:%x:%x:%x:%x:%x:%x:%x, Port: %d\n",
            messageInfo->mPeerAddr.mFields.m16[0],
            messageInfo->mPeerAddr.mFields.m16[1],
            messageInfo->mPeerAddr.mFields.m16[2],
            messageInfo->mPeerAddr.mFields.m16[3],
            messageInfo->mPeerAddr.mFields.m16[4],
            messageInfo->mPeerAddr.mFields.m16[5],
            messageInfo->mPeerAddr.mFields.m16[6],
            messageInfo->mPeerAddr.mFields.m16[7],
            messageInfo->mPeerPort);
    }
}
static void CoapRequestHandler(void *context, otMessage *message, const otMessageInfo *messageInfo) 
{
    printk("CoAP request handler triggered.\n");

    // Extract the token from the incoming request
    const uint8_t* token = otCoapMessageGetToken(message);
    uint8_t tokenLength = otCoapMessageGetTokenLength(message);
    printk("Token: ");
    for (uint8_t i = 0; i < tokenLength; i++) 
    {
        printk("%02X ", token[i]);
    }
    printk("\n");

    // Log the method code
    otCoapCode methodCode = otCoapMessageGetCode(message);
    printk("CoAP Method Code: %d\n", methodCode);

    // Check the length of the message payload
    uint16_t messageLength = otMessageGetLength(message);
    uint16_t offset = otMessageGetOffset(message);
    printk("Message Length: %u, Offset: %u\n", messageLength, offset);

    if (messageLength > offset) 
	{
        char payload[128];
        int length = otMessageRead(message, offset, payload, sizeof(payload) - 1);

        if (length > 0) 
		{
            payload[length] = '\0'; // Null-terminate the payload
            printk("Received CoAP payload: %s\n", payload);

            if (methodCode == OT_COAP_CODE_POST) 
			{
                printk("Handling POST request.\n");
                static char storedData[128];
                strncpy(storedData, payload, sizeof(storedData) - 1);
                storedData[sizeof(storedData) - 1] = '\0'; // Null-terminate
                printk("Stored POST payload: %s\n", storedData);

                if (strcmp(payload, "trigger_action") == 0) {
                    printk("Triggered a specific action based on POST payload.\n");
                }

            } 
			else if (methodCode == OT_COAP_CODE_PUT) 
			{
                printk("Handling PUT request.\n");

                if (strcmp(payload, "all_on") == 0) 
                {
                    printk("All the lamps are on.\n");
                    verzenden = 0xff01;
                    sending = true;
                    // Signal task2 to start processing
                    k_sem_give(&sem_task2_start);
                } 
                else if (strcmp(payload, "all_off") == 0) 
                {
                    printk("All the lamps are off.\n");
                    verzenden = 0xff00;
                    sending = true;
                    // Signal task2 to start processing
                    k_sem_give(&sem_task2_start);
                } 
                else 
                {
                    printk("Unrecognized PUT payload: %s\n", payload);
                    sscanf(payload, "%x", &verzenden);
                    sending = true;
                    // Signal task2 to start processing
                    k_sem_give(&sem_task2_start);
                }
            }
        } 
		else 
		{
            printk("Payload read failed or length is 0.\n");
        }
    } 
	else 
	{
        printk("No payload in the CoAP request.\n");
    }

    // Create and send the response
    otMessage *response = otCoapNewMessage((otInstance *)context, NULL);
    if (response == NULL) 
    {
        printk("Failed to allocate CoAP response message.\n");
        return;
    }

    uint16_t messageId = otCoapMessageGetMessageId(message);
    printk("Incoming CoAP Message ID: %u\n", messageId);

    // Set response type, code, and token
    otCoapMessageInitResponse(response, message, OT_COAP_TYPE_ACKNOWLEDGMENT, OT_COAP_CODE_CHANGED);

    otCoapMessageSetToken(response, token, tokenLength); // Set correct token
    printk("Token: ");

    for (uint8_t i = 0; i < tokenLength; i++) 
    {
        printk("%02X ", token[i]);
    }
    printk("\n");

    otError error = otCoapMessageSetPayloadMarker(response);
    if (error != OT_ERROR_NONE)
    {
        printk("Failed to set payload marker for CoAP response: %s", otThreadErrorToString(error));
        otMessageFree(response);
    }

    uint16_t responseMessageId = otCoapMessageGetMessageId(response);
    printk("Response CoAP Message ID: %u\n", responseMessageId);


    // Append response payload (optional)
    error = otMessageAppend(response, &ontvangen, sizeof(ontvangen));
    if (error != OT_ERROR_NONE) 
    {
        printk("Failed to append payload to CoAP response. Error: %d\n", error);
        otMessageFree(response); // Free response on failure
        return;
    }

    // Send the response
    error = otCoapSendResponse((otInstance *)context, response, messageInfo);
    if (error == OT_ERROR_NONE) 
    {
        printk("CoAP response sent successfully.\n");
    } 
    else 
    {
        printk("Failed to send CoAP response. Error: %d\n", error);
        otMessageFree(response); // Free response on failure
    }
}

void StartNet(otInstance* instance)
{
    otError error1 = otIp6SetEnabled(instance, true);  // Enable IPv6
    if (error1 == OT_ERROR_NONE) 
	{
        printf("IPv6 opgezet\n");
    }
    else 
	{
        printf("Failed to set up IPv6. Error: %d\n", error1);
    }

    otError error2 = otThreadSetEnabled(instance, true);  // Start Thread
    if (error2 == OT_ERROR_NONE) 
	{
        printf("Thread succesvol opgezet\n");
    }
    else 
	{
        printf("Failed to set up Thread. Error: %d\n", error2);
    }
}

/*Data samenvoegen*/
uint16_t Tx_data(uint8_t adres, uint8_t data, bool groep)
{
        uint16_t verzenden;
        verzenden = adres;
        verzenden = verzenden << 8;
        verzenden = verzenden | data;

        if (groep == true)
        {
                verzenden = verzenden | 0x8000;
        }

        return verzenden;
}

/*Delay*/
void SysTickDelay(uint32_t microseconds) 
{
        while (microseconds--) 
        {
                // Wait until the COUNTFLAG is set
                while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) {
                // Do nothing, just wait
                }
                // Reset the SysTick counter value
                SysTick->VAL = 0;
        }
}

/*Data sturen*/
void Tx_data_send(uint16_t verzenden)
{
        //DALI heeft manchester codering positieve flank is 1
        //Uitsturen is manchester codering negatieve flank is 1
        //Ontvangen is manchester codering negatieve flank is 1
    while (sending == true)
    {
        /*startbit sturen*/
        NRF_P1->OUTSET = (1 << 8);
        SysTickDelay(370);
        NRF_P1->OUTCLR = (1 << 8);                                                                
        SysTickDelay(370);

        /*data sturen*/
        for (int i = 15; i>=0; i--)
        {
            uint16_t verzenden1;
            verzenden1 = verzenden >> i;
            verzenden1 = verzenden1 & 0x01;
            
            if (verzenden1 == 0x01)
            {
                    NRF_P1->OUTSET = (1 << 8);
                    SysTickDelay(370);
                    NRF_P1->OUTCLR = (1 << 8);
                    SysTickDelay(370);
            }
            else
            {
                    NRF_P1->OUTCLR = (1 << 8);
                    SysTickDelay(370);
                    NRF_P1->OUTSET = (1 << 8);
                    SysTickDelay(370);  
            }

        }
        
        /*stopbits sturen*/
        NRF_P1->OUTCLR = (1 << 8);
        SysTickDelay(1450);
        NRF_P1->OUTCLR = (1 << 8);
        receive = true;                                             //toelaten backward frame ontvangen. Tussen de 7 half bit en 22 half bittijd wachten.
        sending = false;
        // Signal task1 to execute
        k_sem_give(&sem_task_sync);
        printk("%x is verzonden\n", verzenden);
    }

    return;
}

/*Data ontvangen*/

#define MAX_WAIT_TIME 2000  // Adjust as needed (20ms)
uint8_t Rx_data_receive(bool receive)
{
    uint16_t gekregen = 0;
    Data_received = false;
    uint32_t idleCounter = 0;  // Timeout tracking
    while (receive)
        {
            switch(Verloop)
            {
                    case Idle:
                        counter = 0;
                        gekregen = 0;
                        printk("In Idle\n");
                        if (nrf_gpio_pin_read(P1_07)==1)
                        {
                                Verloop = DetectStartbit1;
                        }
                        else if (nrf_gpio_pin_read(P1_07)==0)
                        {
                            idleCounter++;  // Track time stuck in Idle
                            if (idleCounter > MAX_WAIT_TIME)  
                            {
                                printk("No reply detected! Exiting Idle state.\n");
                                receive = false;  // Exit function gracefully
                            }
                            Verloop = Idle;
                        }
                        break;
    
                    case DetectStartbit1:
                    k_busy_wait(416);
                    printk("In Detectstartbit1\n");
                        if (nrf_gpio_pin_read(P1_07)==0)
                        {
                            uint32_t idleCounter = 0;  // Timeout tracking
                            Verloop = DetectStartbit2;
                        }
                        else
                        {
                            Verloop = Idle;
                        }
                        break;
    
                    case DetectStartbit2:
                    k_busy_wait(416);
                    printk("In DetectStartbit2\n");
                        Verloop = ReadBit;
                        break;
    
                    case ReadBit:
                    printk("In %s", Verloop);
                        if (nrf_gpio_pin_read(P1_07)==0)
                        {
                            Verloop = Delay1;
                        }
                        else if (nrf_gpio_pin_read(P1_07)==1)
                        {
                            Verloop = Delay2;
                        }
                        break;
    
                    case Delay1:
                    k_busy_wait(416);
                    printk("In Delay1\n");
                        if (nrf_gpio_pin_read(P1_07)==1)
                        {
                            gekregen = (gekregen << 1) | 0x00;
                            Verloop = Delay3;
                        }
                        else
                        {
                            Verloop = Idle;
                        }
                        break;
    
                    case Delay2:
                    k_busy_wait(416);
                    printk("In Delay2\n");
                        if (nrf_gpio_pin_read(P1_07)==0)
                        {
                            gekregen = (gekregen << 1) | 0x01;
                            Verloop = Delay3;
                        }
                        else
                        {
                            Verloop = Idle;
                        }
                        break;
    
                    case Delay3:
                    k_busy_wait(416);
                    printk("In Delay3\n");
                        counter++;
                        if (counter < 8)
                        {
                            Verloop = ReadBit;
                        }
                        else
                        {
                            Verloop = Stopbit;
                        }
                        break;
    
                    case Stopbit:
                    k_busy_wait(16666);
                    printk("In Stopbit\n");
                        counter = 0;
                        receive = false;
                        sending = true;
                        Verloop = Idle;
                        Data_received = true;
                        printk("De ontvangen data is: %x\n", gekregen);
                        break;  
            }
        }
    
    
    return gekregen;
}


void StartUDP(otInstance* instance)
{
    // Open UDP socket
    otError error = otUdpOpen(instance, &udpSocket, UdpReceiveCallback, NULL);
    if (error != OT_ERROR_NONE) 
	{
        printf("Failed to open UDP socket. Error: %d\n", error);
        //return -1;
    }

    // Bind the socket to a local port (e.g., 1234)
    otSockAddr localAddr;
    memset(&localAddr, 0, sizeof(localAddr));
    localAddr.mPort = 5683; // Local port
    error = otUdpBind(instance, &udpSocket, &localAddr, OT_NETIF_THREAD);
    if (error != OT_ERROR_NONE) 
	{
        printf("Failed to bind UDP socket. Error: %d\n", error);
    }
    printf("UDP socket bound to local port 5683.\n");
}

void SendUdpMessage(otInstance* instance, const char* payload, const char* ipAddress, uint16_t port) 
{
    otMessage* message;
    otMessageInfo messageInfo;
    // Create a new UDP message
    message = otUdpNewMessage(instance, NULL);
    if (message == NULL) 
	{
        printf("Failed to allocate UDP message.\n");
        return;
    }
    // Append the payload to the message
    otError error = otMessageAppend(message, payload, strlen(payload));
    if (error != OT_ERROR_NONE) 
	{
        printf("Failed to append payload to UDP message. Error: %d\n", error);
        otMessageFree(message); // Free the message on failure
        return;
    }
    // Set up the message info (destination address and port)
    memset(&messageInfo, 0, sizeof(messageInfo));
    messageInfo.mPeerPort = port;
    otIp6AddressFromString(ipAddress, &messageInfo.mPeerAddr);
    // Send the message
    error = otUdpSend(instance, &udpSocket, message, &messageInfo);
    if (error == OT_ERROR_NONE) 
	{
        printf("UDP message sent successfully: %s\n", payload);
    }
    else 
	{
        printf("Failed to send UDP message. Error: %d\n", error);
        otMessageFree(message); // Free the message on failure
    }
}

