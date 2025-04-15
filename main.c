/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
/*Includes OpenThread, UDP*/
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

/*Includes DALI*/
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

otInstance *instance;
otUdpSocket udpSocket;
otOperationalDataset dataset;
LOG_MODULE_REGISTER(cli_sample, CONFIG_OT_COMMAND_LINE_INTERFACE_LOG_LEVEL);

/*Taken defines*/
#define STACK_SIZE 1024
#define THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(thread_stack_1, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_stack_2, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_stack_3, STACK_SIZE);

struct k_thread thread_data_1;
struct k_thread thread_data_2;
struct k_thread thread_data_3;

// Semaphore to synchronize task1 and task2
struct k_sem sem_task_sync;

// Declare and initialize the mutex
struct k_mutex my_mutex;

/*Taak intervallen*/
uint32_t task1_interval_us = 1000; // Task 1 interval in microseconds (1 second) 10000 us voor 1*8bits + 1 startbit + 2 stopbits
uint32_t task2_interval_us = 9163;  // Task 2 interval in microseconds (0.5 seconds)
uint32_t task3_interval_us = 2000;  // Task 2 interval in microseconds (0.5 seconds)

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
static verloop Verloop = Idle;
static uint8_t counter = 0;
uint8_t teller = 0;
static uint16_t ontvangen = 0;
uint16_t verzenden = 0;                 //variabele waar alles in wordt toegevoegd
bool receive = false;
bool send = false;
bool Data_received = false;  
static const uint8_t extendedPanId[] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};
static const uint8_t networkKey[] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

/*Functies initialiseren*/
uint16_t Tx_data(uint8_t, uint8_t, bool);
void half_bit(void);
void Tx_data_send(uint16_t);
void SysTick_Init(void);
void SysTickDelay(uint32_t);
uint8_t Rx_data_receive(bool);
bool wait_for_next_frame(bool);
void init_openthread(void);
void configure_thread_network(void);
void send_udp_message(void);
void StartNet(otInstance*);
void UdpReceiveCallback(void*, otMessage*, const otMessageInfo*);
void StartUDP(otInstance*);
void SendUdpMessage(otInstance*, const char*, const char*, uint16_t);
 
/*Taken*/
void task1(void *arg1, void *arg2, void *arg3) 
{
    while (1) {
		k_sem_take(&sem_task_sync, K_FOREVER);
		if (k_mutex_lock(&my_mutex, K_FOREVER) == 0) 
		{
			//printk("Task 1 mutex locked.\n");
			// Wait for task2 to signal execution
			
			k_usleep(2900);
			ontvangen = Rx_data_receive(receive);

			k_mutex_unlock(&my_mutex);
			//printk("Task 1 mutex unlocked.\n");
        } 
		else 
		{
            printk("Task 1 failed to lock the mutex.\n");
        }

        k_usleep(task1_interval_us);
    }
    
}

void task2(void *arg1, void *arg2, void *arg3) 
{
    while (1) {
		if (k_mutex_lock(&my_mutex, K_FOREVER) == 0) 
		{
            //send = true;
			//printk("Task 2 mutex locked.\n");
			if (Data_received == true)
			{
				k_usleep(9163);
			}
			else if (Data_received == false)
			{
				k_usleep(6263);
			}
			/*Te versturen data. Nog taak maken om dit in te lezen*/
			uint8_t adres = 0x7f; //eigenlijk maar 7 bit
			uint8_t data = 0x00;
			bool groep = true;


			verzenden = Tx_data(adres, data, groep);                     //data samenvoegen
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

void task3(void *arg1, void *arg2, void *arg3) 
{
	while (1) {
		if (k_mutex_lock(&my_mutex, K_FOREVER) == 0) 
		{
			//printk("Task 3 mutex locked.\n");
             
        	StartUDP(instance);

			k_mutex_unlock(&my_mutex);
			//printk("Task 3 mutex unlocked.\n");
		} 
		else 
		{
			printk("Task 3 failed to lock the mutex.\n");
		}	
        k_usleep(task3_interval_us);
    }
	
}

void DALI_Discover_Devices(void)
{
    for (uint8_t address = 0x00; address <= 0x7F; address++) {
        uint16_t queryCommand = Tx_data(address, 0x90, false); // Query Status command
        send = true;
        Tx_data_send(queryCommand); // Send command
        Rx_data_receive(receive);
        k_usleep(20000); // Wait for backward frame

        if (Data_received) {
            printk("Device found at address: 0x%02x\n", address);
        } else {
            printk("No response from address: 0x%02x\n", address);
        }

        Data_received = false; // Reset flag
    }
    uint16_t queryCommand = Tx_data(0x7f, 0x00, true); // Query Status command
    send = true;
    Tx_data_send(queryCommand); // Send command
    Rx_data_receive(receive);
    if (Data_received) {
        printk("Device found at address: 0x%02x\n", 255);
    } else {
        printk("No response from address: 0x%02x\n", 255);
    }

    Data_received = false; // Reset flag
}

void DALI_Bus_Init(void)
{
    // Set output pin for DALI bus to idle state (logic 1)
    nrf_gpio_cfg_output(P1_08);
    nrf_gpio_pin_set(P1_08); // Logic 1 (idle state)

    // Set input pin for backward frames
    nrf_gpio_cfg_input(P1_07, NRF_GPIO_PIN_NOPULL);

    // Print confirmation
    printk("DALI bus initialized and idle.\n");
}

int main(void)
{
    // Initialize the GPIO for DALI communication
    DALI_Bus_Init();

    instance = otInstanceInitSingle();
    memset(&dataset, 0, sizeof(dataset));

	k_mutex_init(&my_mutex);
    k_sem_init(&sem_task_sync, 0, 1);
    SysTick_Init();
    
	/*connect to existing network*/
	StartNet(instance);
	//StartUDP(instance);

    // Discover connected DALI devices
    DALI_Discover_Devices();

    k_tid_t task1_tid = k_thread_create(&thread_data_1, thread_stack_1, STACK_SIZE,
                                        task1, NULL, NULL, NULL,
                                        THREAD_PRIORITY, 0, K_NO_WAIT);

    k_tid_t task2_tid = k_thread_create(&thread_data_2, thread_stack_2, STACK_SIZE,
                                        task2, NULL, NULL, NULL,
                                        THREAD_PRIORITY, 0, K_NO_WAIT);
	
	k_tid_t task3_tid = k_thread_create(&thread_data_3, thread_stack_3, STACK_SIZE,
										task3, NULL, NULL, NULL,
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

/*Opstarten IPv6 en Thread*/
void StartNet(otInstance* instance)
{
    dataset.mActiveTimestamp.mSeconds = 1;
    dataset.mChannel = 15;
    dataset.mPanId = 0x1234;
    memcpy(dataset.mExtendedPanId.m8, extendedPanId, sizeof(dataset.mExtendedPanId.m8));
    memcpy(dataset.mNetworkKey.m8, networkKey, sizeof(dataset.mNetworkKey.m8));

    otError error1 = otIp6SetEnabled(instance, true);  // Enable IPv6
    if (error1 == OT_ERROR_NONE) {
        printf("IPv6 opgezet\n");
    }
    else {
        printf("Failed to set up IPv6. Error: %d\n", error1);
    }
    otError error2 = otThreadSetEnabled(instance, true);  // Start Thread
    if (error2 == OT_ERROR_NONE) {
        printf("Thread succesvol opgezet\n");
    }
    else {
        printf("Failed to set up Thread. Error: %d\n", error2);
    }
}

/*Initialisatie SysTick*/
void SysTick_Init(void) 
{
        // SystemCoreClock is typically set to 64 MHz on the nRF52840
        SysTick->LOAD = (SystemCoreClock / 1000000) - 1;  // Load with number of clocks per microsecond
        SysTick->VAL = 0;  // Reset the SysTick counter value
        SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;  // Enable SysTick Timer without interrupt
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
    while (send == true)
    {
        /*startbit sturen*/
        NRF_P1->OUTSET = (1 << 8);
        //SysTickDelay(365);
        k_busy_wait(245);
        NRF_P1->OUTCLR = (1 << 8);                                                                
        //SysTickDelay(365);
        k_busy_wait(585);

        /*data sturen*/
        for (int i = 15; i>=0; i--)
        {
            uint16_t verzenden1;
            verzenden1 = verzenden >> i;
            verzenden1 = verzenden1 & 0x01;
            
            if (verzenden1 == 0x01)
            {
                    NRF_P1->OUTSET = (1 << 8);
                    //SysTickDelay(365);
                    k_busy_wait(245);
                    NRF_P1->OUTCLR = (1 << 8);
                    k_busy_wait(585);
                    //SysTickDelay(365);
            }
            else
            {
                    NRF_P1->OUTCLR = (1 << 8);
                    k_busy_wait(585);
                    //SysTickDelay(365);
                    NRF_P1->OUTSET = (1 << 8);
                    k_busy_wait(245);
                    //SysTickDelay(365);  
            }

        }
        
        /*stopbits sturen*/
        NRF_P1->OUTCLR = (1 << 8);
        k_busy_wait(16666);
        //SysTickDelay(1450);
        NRF_P1->OUTCLR = (1 << 8);
        receive = true;                                             //toelaten backward frame ontvangen. Tussen de 7 half bit en 22 half bittijd wachten.
        send = false;
        // Signal task1 to execute
        k_sem_give(&sem_task_sync);
        printk("%x is verzonden\n", verzenden);
    }

    return;
}


/*Data ontvangen*/
uint8_t Rx_data_receive(bool receive)
{
    uint16_t gekregen = 0;
    Data_received = false;
    uint32_t timeout = 1000; // Example timeout to prevent infinite loop

    while ((receive == true)  && timeout--)
        {
            
            switch(Verloop)
            {
                    case Idle:
                        counter = 0;
                        gekregen = 0;
                        printk("State: Idle\n");
                        if (nrf_gpio_pin_read(P1_07)==1)
                        {
                                Verloop = DetectStartbit1;
                        }
                        else
                        {
                                Verloop = Idle;
                        }
                        break;
    
                    case DetectStartbit1:
                        SysTickDelay(365);
                        printk("State: DetectStartbit1\n");
                        if (nrf_gpio_pin_read(P1_07)==0)
                        {
                            Verloop = DetectStartbit2;
                        }
                        else
                        {
                            Verloop = Idle;
                        }
                        break;
    
                    case DetectStartbit2:
                        SysTickDelay(365);
                        printk("State: DetectStartbit2\n");
                        Verloop = ReadBit;
                        break;
    
                    case ReadBit:
                    printk("State: ReadBit\n");
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
                        SysTickDelay(365);
                        printk("State: Delay1\n");
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
                        SysTickDelay(365);
                        printk("State: Delay2\n");
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
                        SysTickDelay(365);
                        printk("State: Delay3\n");
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
                        SysTickDelay(1450);
                        printk("State: Stopbit\n");
                        counter = 0;
                        receive = false;
                        send = true;
                        Verloop = Idle;
                        Data_received = true;
                        printk("De ontvangen data is: %x\n", gekregen);
						char recu[6]; // Buffer to hold the string version of the port
						sprintf(recu, "%u", gekregen); // Convert uint16_t to string
						SendUdpMessage(instance, recu, "ff02::1", 2345);
                        break;  
            }
			
        }
        if (timeout == 0) {
            printk("Timeout occurred, resetting state machine.\n");
            Verloop = Idle;
            receive = false;
        }
	
    return gekregen;
}
// Callback function for receiving UDP messages
void UdpReceiveCallback(void* context, otMessage* message, const otMessageInfo* messageInfo) 
{
    char buffer[128]; // Buffer for the incoming message
    int length = otMessageRead(message, otMessageGetOffset(message), buffer, sizeof(buffer) - 1);

    if (length > 0) {
        buffer[length] = '\0'; // Null-terminate the received message
        printf("Received UDP message: %s\n", buffer);
		verzenden = (uint16_t)atoi(buffer);
		send = true;
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

/*UDP opstarten*/
void StartUDP(otInstance* instance)
{
    // Open UDP socket
    otError error = otUdpOpen(instance, &udpSocket, UdpReceiveCallback, NULL);
    if (error != OT_ERROR_NONE) {
        printf("Failed to open UDP socket. Error: %d\n", error);
    }

    // Bind the socket to a local port (e.g., 1234)
    otSockAddr localAddr;
    memset(&localAddr, 0, sizeof(localAddr));
    localAddr.mPort = 1234; // Local port
    error = otUdpBind(instance, &udpSocket, &localAddr, OT_NETIF_THREAD);
    if (error != OT_ERROR_NONE) {
        printf("Failed to bind UDP socket. Error: %d\n", error);
    }
    //printf("UDP socket bound to local port 1234.\n");
}

/*UDP sturen*/
void SendUdpMessage(otInstance* instance, const char* payload, const char* ipAddress, uint16_t port) 
{
    otMessage* message;
    otMessageInfo messageInfo;
    // Create a new UDP message
    message = otUdpNewMessage(instance, NULL);
    if (message == NULL) {
        printf("Failed to allocate UDP message.\n");
        return;
    }
    // Append the payload to the message
    otError error = otMessageAppend(message, payload, strlen(payload));
    if (error != OT_ERROR_NONE) {
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
    if (error == OT_ERROR_NONE) {
        //printf("UDP message sent successfully: %s\n", payload);
    }
    else {
        //printf("Failed to send UDP message. Error: %d\n", error);
        otMessageFree(message); // Free the message on failure
    }
}