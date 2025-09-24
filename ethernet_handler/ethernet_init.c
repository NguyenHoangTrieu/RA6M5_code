#include "ethernet_init.h"
#include "FreeRTOS_DHCP.h"
#include "FreeRTOS_DNS.h"
#include <stdio.h>
#include <string.h>

// Static variables
static ethernet_config_t eth_config;
static ethernet_status_t eth_status;
static uint8_t ethernet_initialized = 0;
static uint8_t ethernet_started = 0;

// FSP handles
static ether_instance_ctrl_t g_ether0_ctrl;
static ether_phy_instance_ctrl_t g_ether_phy0_ctrl;

// Callback functions
static ethernet_link_callback_t link_callback = NULL;
static ethernet_dhcp_callback_t dhcp_callback = NULL;

// FreeRTOS+TCP network buffer configuration
static NetworkBufferDescriptor_t *pxBufferDescriptors;
static uint8_t *pucNetworkBuffers;

// Default configuration for CK-RA6M5
static void ethernet_init_default_config(void) {
    // Default MAC address (should be unique for production)
    eth_config.mac_address[0] = 0x00;
    eth_config.mac_address[1] = 0x11;
    eth_config.mac_address[2] = 0x22;
    eth_config.mac_address[3] = 0x33;
    eth_config.mac_address[4] = 0x44;
    eth_config.mac_address[5] = 0x55;
    
    // Network configuration
    eth_config.use_dhcp = 1;  // Use DHCP by default
    eth_config.ip_address = 0;
    eth_config.netmask = 0;
    eth_config.gateway = 0;
    eth_config.dns_server = 0;
    
    // PHY configuration
    eth_config.link_speed = ETH_LINK_SPEED_100F;
    eth_config.duplex_mode = ETH_LINK_DUPLEX_FULL;
    eth_config.phy_reset_time_ms = 500;
    
    // Initialize status
    memset(&eth_status, 0, sizeof(eth_status));
}

// PHY callback for link status changes
void ethernet_phy_callback(ether_phy_callback_args_t *p_args) {
    if (p_args->event == ETHER_PHY_EVENT_LINK_ON) {
        eth_status.link_up = 1;
        printf("Ethernet: Link UP\n");
        
        if (link_callback) {
            link_callback(1);
        }
    } else if (p_args->event == ETHER_PHY_EVENT_LINK_OFF) {
        eth_status.link_up = 0;
        eth_status.dhcp_acquired = 0;
        printf("Ethernet: Link DOWN\n");
        
        if (link_callback) {
            link_callback(0);
        }
    }
}

// Ethernet controller callback
void ethernet_callback(ether_callback_args_t *p_args) {
    // Handle Ethernet controller events
    switch (p_args->event) {
        case ETHER_EVENT_INTERRUPT:
            // Handle interrupt events
            break;
        default:
            break;
    }
}

// Initialize network buffers for FreeRTOS+TCP
static int ethernet_init_network_buffers(void) {
    size_t buffer_size = sizeof(NetworkBufferDescriptor_t) * ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS;
    size_t network_buffer_size = ipTOTAL_BUFFER_SIZE * ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS;
    
    // Allocate buffer descriptors
    pxBufferDescriptors = (NetworkBufferDescriptor_t*)pvPortMalloc(buffer_size);
    if (!pxBufferDescriptors) {
        printf("ERROR: Failed to allocate buffer descriptors\n");
        return -1;
    }
    
    // Allocate network buffers
    pucNetworkBuffers = (uint8_t*)pvPortMalloc(network_buffer_size);
    if (!pucNetworkBuffers) {
        vPortFree(pxBufferDescriptors);
        printf("ERROR: Failed to allocate network buffers\n");
        return -1;
    }
    
    // Initialize buffer descriptors
    for (int i = 0; i < ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS; i++) {
        pxBufferDescriptors[i].pucEthernetBuffer = &pucNetworkBuffers[i * ipTOTAL_BUFFER_SIZE];
        pxBufferDescriptors[i].xDataLength = 0;
        pxBufferDescriptors[i].pxNextBuffer = (i < ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS - 1) ? 
                                             &pxBufferDescriptors[i + 1] : NULL;
    }
    
    return 0;
}

// Initialize Ethernet system
int ethernet_init(void) {
    if (ethernet_initialized) {
        printf("Ethernet: Already initialized\n");
        return 0;
    }
    
    // Initialize default configuration
    ethernet_init_default_config();
    
    // Initialize network buffers
    if (ethernet_init_network_buffers() != 0) {
        return -1;
    }
    
    fsp_err_t err;
    
    // Open Ethernet PHY
    err = R_ETHER_PHY_Open(&g_ether_phy0_ctrl, &g_ether_phy0_cfg);
    if (FSP_SUCCESS != err) {
        printf("ERROR: Ethernet PHY open failed: %d\n", err);
        return -1;
    }
    
    // Open Ethernet controller
    err = R_ETHER_Open(&g_ether0_ctrl, &g_ether0_cfg);
    if (FSP_SUCCESS != err) {
        printf("ERROR: Ethernet controller open failed: %d\n", err);
        R_ETHER_PHY_Close(&g_ether_phy0_ctrl);
        return -1;
    }
    
    // Set MAC address
    err = R_ETHER_CallbackSet(&g_ether0_ctrl, ethernet_callback, NULL, NULL);
    if (FSP_SUCCESS != err) {
        printf("WARNING: Failed to set Ethernet callback: %d\n", err);
    }
    
    printf("Ethernet: Initialized successfully\n");
    ethernet_initialized = 1;
    return 0;
}

// Start Ethernet interface
int ethernet_start(void) {
    if (!ethernet_initialized) {
        printf("ERROR: Ethernet not initialized\n");
        return -1;
    }
    
    if (ethernet_started) {
        printf("Ethernet: Already started\n");
        return 0;
    }
    
    // Initialize FreeRTOS+TCP network interface
    NetworkInterface_t *pxInterface = FreeRTOS_GetNetworkInterface();
    if (!pxInterface) {
        printf("ERROR: Failed to get network interface\n");
        return -1;
    }
    
    // Set up network parameters
    uint32_t ip_addr = eth_config.use_dhcp ? 0 : eth_config.ip_address;
    uint32_t netmask = eth_config.use_dhcp ? 0 : eth_config.netmask;
    uint32_t gateway = eth_config.use_dhcp ? 0 : eth_config.gateway;
    uint32_t dns = eth_config.use_dhcp ? 0 : eth_config.dns_server;
    
    // Initialize the TCP/IP stack
    if (FreeRTOS_IPInit(eth_config.mac_address, ip_addr, netmask, gateway, dns) != pdPASS) {
        printf("ERROR: FreeRTOS+TCP initialization failed\n");
        return -1;
    }
    
    // Start the PHY
    fsp_err_t err = R_ETHER_PHY_StartAutoNegotiate(&g_ether_phy0_ctrl);
    if (FSP_SUCCESS != err) {
        printf("ERROR: PHY auto-negotiation start failed: %d\n", err);
        return -1;
    }
    
    printf("Ethernet: Started successfully\n");
    ethernet_started = 1;
    return 0;
}

// Stop Ethernet interface
int ethernet_stop(void) {
    if (!ethernet_started) {
        return 0;
    }
    
    // Stop network interface
    FreeRTOS_NetworkDown();
    
    // Reset status
    memset(&eth_status, 0, sizeof(eth_status));
    
    printf("Ethernet: Stopped\n");
    ethernet_started = 0;
    return 0;
}

// Deinitialize Ethernet system
void ethernet_deinit(void) {
    if (!ethernet_initialized) {
        return;
    }
    
    ethernet_stop();
    
    // Close Ethernet controller and PHY
    R_ETHER_Close(&g_ether0_ctrl);
    R_ETHER_PHY_Close(&g_ether_phy0_ctrl);
    
    // Free network buffers
    if (pxBufferDescriptors) {
        vPortFree(pxBufferDescriptors);
        pxBufferDescriptors = NULL;
    }
    
    if (pucNetworkBuffers) {
        vPortFree(pucNetworkBuffers);
        pucNetworkBuffers = NULL;
    }
    
    ethernet_initialized = 0;
    printf("Ethernet: Deinitialized\n");
}

// Configuration and status functions
const ethernet_config_t* ethernet_get_config(void) {
    return &eth_config;
}

int ethernet_set_config(const ethernet_config_t* config) {
    if (!config) {
        return -1;
    }
    
    memcpy(&eth_config, config, sizeof(ethernet_config_t));
    return 0;
}

const ethernet_status_t* ethernet_get_status(void) {
    // Update current IP information
    if (ethernet_started) {
        eth_status.assigned_ip = FreeRTOS_GetIPAddress();
        eth_status.assigned_netmask = FreeRTOS_GetNetmask();
        eth_status.assigned_gateway = FreeRTOS_GetGatewayAddress();
        eth_status.dhcp_acquired = (eth_status.assigned_ip != 0);
    }
    
    return &eth_status;
}

// Network interface functions
uint32_t ethernet_get_ip_address(void) {
    return FreeRTOS_GetIPAddress();
}

uint32_t ethernet_get_netmask(void) {
    return FreeRTOS_GetNetmask();
}

uint32_t ethernet_get_gateway(void) {
    return FreeRTOS_GetGatewayAddress();
}

const uint8_t* ethernet_get_mac_address(void) {
    return eth_config.mac_address;
}

// Link status functions
uint8_t ethernet_is_link_up(void) {
    return eth_status.link_up;
}

uint8_t ethernet_is_dhcp_ready(void) {
    return eth_status.dhcp_acquired;
}

int ethernet_wait_for_link(uint32_t timeout_ms) {
    TickType_t start_time = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    
    while (!ethernet_is_link_up()) {
        if ((xTaskGetTickCount() - start_time) > timeout_ticks) {
            return -1; // Timeout
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    return 0; // Success
}

int ethernet_wait_for_dhcp(uint32_t timeout_ms) {
    TickType_t start_time = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    
    while (!ethernet_is_dhcp_ready()) {
        if ((xTaskGetTickCount() - start_time) > timeout_ticks) {
            return -1; // Timeout
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    return 0; // Success
}

// Utility functions
void ethernet_get_ip_string(char* ip_str, size_t str_len) {
    if (!ip_str || str_len < 16) return;
    
    uint32_t ip = ethernet_get_ip_address();
    snprintf(ip_str, str_len, "%d.%d.%d.%d",
             (int)(ip & 0xFF),
             (int)((ip >> 8) & 0xFF),
             (int)((ip >> 16) & 0xFF),
             (int)((ip >> 24) & 0xFF));
}

void ethernet_get_mac_string(char* mac_str, size_t str_len) {
    if (!mac_str || str_len < 18) return;
    
    const uint8_t* mac = ethernet_get_mac_address();
    snprintf(mac_str, str_len, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// Callback registration
int ethernet_register_link_callback(ethernet_link_callback_t callback) {
    link_callback = callback;
    return 0;
}

int ethernet_register_dhcp_callback(ethernet_dhcp_callback_t callback) {
    dhcp_callback = callback;
    return 0;
}

// Hook functions for FreeRTOS+TCP integration
void vApplicationIPNetworkEventHook(eIPCallbackEvent_t eNetworkEvent) {
    switch (eNetworkEvent) {
        case eNetworkUp:
            printf("Network: Interface UP\n");
            eth_status.dhcp_acquired = 1;
            if (dhcp_callback) {
                dhcp_callback(FreeRTOS_GetIPAddress());
            }
            break;
            
        case eNetworkDown:
            printf("Network: Interface DOWN\n");
            eth_status.dhcp_acquired = 0;
            break;
            
        default:
            break;
    }
}

BaseType_t xApplicationGetRandomNumber(uint32_t *pulNumber) {
    // Generate random number for network stack
    // In production, use hardware RNG
    *pulNumber = (uint32_t)xTaskGetTickCount();
    return pdTRUE;
}

uint32_t ulApplicationGetNextSequenceNumber(uint32_t ulSourceAddress,
                                           uint16_t usSourcePort,
                                           uint32_t ulDestinationAddress,
                                           uint16_t usDestinationPort) {
    // Generate sequence number for TCP
    return (uint32_t)xTaskGetTickCount();
}