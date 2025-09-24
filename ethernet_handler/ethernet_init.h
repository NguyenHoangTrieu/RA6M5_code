#ifndef ETHERNET_INIT_H
#define ETHERNET_INIT_H

#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "hal_data.h"
#include "r_ether_api.h"
#include "r_ether_phy_api.h"

// Ethernet configuration for CK-RA6M5
typedef struct {
    uint8_t mac_address[6];          // MAC address
    uint32_t ip_address;             // Static IP address (0 for DHCP)
    uint32_t netmask;                // Network mask
    uint32_t gateway;                // Default gateway
    uint32_t dns_server;             // DNS server
    uint8_t use_dhcp;                // 1 = DHCP, 0 = static IP
    uint16_t link_speed;             // ETH_LINK_SPEED_10H/100H/10F/100F
    uint8_t duplex_mode;             // ETH_LINK_DUPLEX_HALF/FULL
    uint32_t phy_reset_time_ms;      // PHY reset time
} ethernet_config_t;

// Ethernet status structure
typedef struct {
    uint8_t link_up;                 // Link status
    uint8_t dhcp_acquired;           // DHCP IP acquired
    uint32_t assigned_ip;            // Current IP address
    uint32_t assigned_netmask;       // Current netmask
    uint32_t assigned_gateway;       // Current gateway
    uint16_t current_link_speed;     // Current link speed
    uint8_t current_duplex;          // Current duplex mode
} ethernet_status_t;

// Link speed definitions
#define ETH_LINK_SPEED_10H    (ETHER_PHY_LINK_SPEED_10H)
#define ETH_LINK_SPEED_10F    (ETHER_PHY_LINK_SPEED_10F)
#define ETH_LINK_SPEED_100H   (ETHER_PHY_LINK_SPEED_100H)
#define ETH_LINK_SPEED_100F   (ETHER_PHY_LINK_SPEED_100F)

// Duplex mode definitions
#define ETH_LINK_DUPLEX_HALF  (ETHER_PHY_DUPLEX_HALF)
#define ETH_LINK_DUPLEX_FULL  (ETHER_PHY_DUPLEX_FULL)

// Function prototypes
int ethernet_init(void);
int ethernet_start(void);
int ethernet_stop(void);
void ethernet_deinit(void);

// Configuration and status functions
const ethernet_config_t* ethernet_get_config(void);
int ethernet_set_config(const ethernet_config_t* config);
const ethernet_status_t* ethernet_get_status(void);

// Network interface functions
uint32_t ethernet_get_ip_address(void);
uint32_t ethernet_get_netmask(void);
uint32_t ethernet_get_gateway(void);
const uint8_t* ethernet_get_mac_address(void);

// Link status functions
uint8_t ethernet_is_link_up(void);
uint8_t ethernet_is_dhcp_ready(void);
int ethernet_wait_for_link(uint32_t timeout_ms);
int ethernet_wait_for_dhcp(uint32_t timeout_ms);

// Utility functions
void ethernet_get_ip_string(char* ip_str, size_t str_len);
void ethernet_get_mac_string(char* mac_str, size_t str_len);

// Callback registration
typedef void (*ethernet_link_callback_t)(uint8_t link_up);
typedef void (*ethernet_dhcp_callback_t)(uint32_t ip_address);

int ethernet_register_link_callback(ethernet_link_callback_t callback);
int ethernet_register_dhcp_callback(ethernet_dhcp_callback_t callback);

#endif // ETHERNET_INIT_H
