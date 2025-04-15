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


otInstance *instance;
otUdpSocket udpSocket;
otOperationalDataset dataset;
static otCoapResource coapResource;

LOG_MODULE_REGISTER(cli_sample, CONFIG_OT_COMMAND_LINE_INTERFACE_LOG_LEVEL);

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

        if (length > 0) {
            payload[length] = '\0'; // Null-terminate the payload
            printk("Received CoAP payload: %s\n", payload);

            if (methodCode == OT_COAP_CODE_POST) 
            {
                printk("Handling POST request.\n");
                static char storedData[128];
                strncpy(storedData, payload, sizeof(storedData) - 1);
                storedData[sizeof(storedData) - 1] = '\0'; // Null-terminate
                printk("Stored POST payload: %s\n", storedData);

                if (strcmp(payload, "trigger_action") == 0) 
                {
                    printk("Triggered a specific action based on POST payload.\n");
                }

            } 
            else if (methodCode == OT_COAP_CODE_PUT) 
            {
                printk("Handling PUT request.\n");

                if (strcmp(payload, "enable_feature") == 0) 
                {
                    printk("Feature enabled.\n");
                } 
                else if (strcmp(payload, "disable_feature") == 0) 
                {
                    printk("Feature disabled.\n");
                } 
                else 
                {
                    printk("Unrecognized PUT payload: %s\n", payload);
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
    error = otMessageAppend(response, "Antwoord verzonden", strlen("Antwoord verzonden"));
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


void InitializeCoap(otInstance *instance) 
{
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

// Callback function for receiving UDP messages
void UdpReceiveCallback(void* context, otMessage* message, const otMessageInfo* messageInfo) {
    char buffer[128]; // Buffer for the incoming message
    int length = otMessageRead(message, otMessageGetOffset(message), buffer, sizeof(buffer) - 1);

    if (length > 0) 
    {
        buffer[length] = '\0'; // Null-terminate the received message
        printf("Received UDP message: %s\n", buffer);
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


int main(void)
{
    instance = otInstanceInitSingle();
    memset(&dataset, 0, sizeof(dataset));
    /*connect to existing network*/
    StartNet(instance);
    StartUDP(instance);
    k_usleep(10000000);

    InitializeCoap(instance);
    otDeviceRole role = otThreadGetDeviceRole(instance);
    printk("Device Role: %d\n", role);
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



