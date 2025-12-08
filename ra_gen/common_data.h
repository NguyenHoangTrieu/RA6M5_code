/* generated common header file - do not edit */
#ifndef COMMON_DATA_H_
#define COMMON_DATA_H_
#include <stdint.h>
#include "bsp_api.h"
#include "r_iic_master.h"
#include "r_i2c_master_api.h"
#include "rm_comms_i2c.h"
#include "rm_comms_api.h"
#include "r_ether_phy.h"
#include "r_ether_phy_api.h"
#include "r_ether.h"
#include "r_ether_api.h"
#include "FreeRTOSIPConfig.h"
#include "rm_freertos_plus_tcp.h"
#include "FreeRTOS_IP.h"
#include "r_ioport.h"
#include "bsp_pin_cfg.h"
FSP_HEADER
/* I2C Master on IIC Instance. */
extern const i2c_master_instance_t g_i2c_master1;

/** Access the I2C Master instance using these structures when calling API functions directly (::p_api is not used). */
extern iic_master_instance_ctrl_t g_i2c_master1_ctrl;
extern const i2c_master_cfg_t g_i2c_master1_cfg;

#ifndef rm_comms_i2c_callback
void rm_comms_i2c_callback(i2c_master_callback_args_t * p_args);
#endif
/* I2C Shared Bus */
extern rm_comms_i2c_bus_extended_cfg_t g_comms_i2c_bus1_extended_cfg;
#ifndef ETHER_PHY_LSI_TYPE_KIT_COMPONENT
  #define ETHER_PHY_LSI_TYPE_KIT_COMPONENT ETHER_PHY_LSI_TYPE_DEFAULT
#endif

#ifndef NULL
void NULL(ether_phy_instance_ctrl_t * p_instance_ctrl);
#endif

#ifndef NULL
bool NULL(ether_phy_instance_ctrl_t * p_instance_ctrl, uint32_t line_speed_duplex);
#endif

/** ether_phy on ether_phy Instance. */
extern const ether_phy_instance_t g_ether_phy0;

/** Access the Ethernet PHY instance using these structures when calling API functions directly (::p_api is not used). */
extern ether_phy_instance_ctrl_t g_ether_phy0_ctrl;
extern const ether_phy_cfg_t g_ether_phy0_cfg;
extern const ether_phy_extended_cfg_t g_ether_phy0_extended_cfg;
#if (BSP_FEATURE_TZ_HAS_TRUSTZONE == 1)  && (BSP_TZ_NONSECURE_BUILD != 1) && (BSP_FEATURE_ETHER_SUPPORTS_TZ_SECURE == 0)
#define ETHER_BUFFER_PLACE_IN_SECTION BSP_PLACE_IN_SECTION(BSP_UNINIT_SECTION_PREFIX ".ns_buffer")
#else
#define ETHER_BUFFER_PLACE_IN_SECTION
#endif

/** ether on ether Instance. */
extern const ether_instance_t g_ether0;

/** Access the Ethernet instance using these structures when calling API functions directly (::p_api is not used). */
extern ether_instance_ctrl_t g_ether0_ctrl;
extern const ether_cfg_t g_ether0_cfg;

#ifndef vEtherISRCallback
void vEtherISRCallback(ether_callback_args_t * p_args);
#endif
#if (ipconfigIPv4_BACKWARD_COMPATIBLE == 1)
 extern rm_freertos_plus_tcp_instance_t * gp_freertos_plus_tcp_instance;
#else
 extern NetworkInterface_t g_freertos_plus_tcp0_xInterface;
 extern NetworkInterface_t * pxFSP_Eth_FillInterfaceDescriptor(BaseType_t xEMACIndex, NetworkInterface_t * pxInterface);
#endif
extern rm_freertos_plus_tcp_instance_t g_freertos_plus_tcp0;
/* I2C Master on IIC Instance. */
extern const i2c_master_instance_t g_i2c_master0;

/** Access the I2C Master instance using these structures when calling API functions directly (::p_api is not used). */
extern iic_master_instance_ctrl_t g_i2c_master0_ctrl;
extern const i2c_master_cfg_t g_i2c_master0_cfg;

#ifndef rm_comms_i2c_callback
void rm_comms_i2c_callback(i2c_master_callback_args_t * p_args);
#endif
/* I2C Shared Bus */
extern rm_comms_i2c_bus_extended_cfg_t g_comms_i2c_bus0_extended_cfg;
#define IOPORT_CFG_NAME g_bsp_pin_cfg
#define IOPORT_CFG_OPEN R_IOPORT_Open
#define IOPORT_CFG_CTRL g_ioport_ctrl

/* IOPORT Instance */
extern const ioport_instance_t g_ioport;

/* IOPORT control structure. */
extern ioport_instance_ctrl_t g_ioport_ctrl;
void g_common_init(void);
FSP_FOOTER
#endif /* COMMON_DATA_H_ */
