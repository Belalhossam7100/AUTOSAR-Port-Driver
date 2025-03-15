 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_PBcfg.c
 *
 * Description: Post Build Configuration Source file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Mohamed Tarek
 ******************************************************************************/
#include "Port.h"

/*
 * Module Version 1.0.0
 */
#define PORT_PBCFG_SW_MAJOR_VERSION              (1U)
#define PORT_PBCFG_SW_MINOR_VERSION              (0U)
#define PORT_PBCFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_PBCFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_PBCFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_PBCFG_AR_RELEASE_PATCH_VERSION     (3U)

/* AUTOSAR Version checking between Port_PBcfg.c and Port.h files */
#if ((PORT_PBCFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_PBCFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_PBCFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Port_PBcfg.c does not match the expected version"
#endif

/* Software Version checking between Port_PBcfg.c and Port.h files */
#if ((PORT_PBCFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_PBCFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_PBCFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of Port_PBcfg.c does not match the expected version"
#endif
/*************************************************/
/* PB Configuration Structure for the LED using Macro*/
const Port_ConfigType LED_PORT_Configuration ={
    .port_num = LED_PORT_NUM,          /* Set port value using macro */
    .pin_num = LED_PIN_NUM,            /* Set pin number value using macro */
    .pin_mode = LED_PIN_MODE,          /* Set Pin Mode using macro */
    .direction = LED_PIN_DIRECTION,    /* Set pin as OUTPUT using macro */
    .resistor = LED_PIN_RESISTOR,      /* Disable internal resistor using macro */
    .initial_value = LED_INITIAL_VALUE /* Initial value (LED off, for example) using macro */
};
/*************************************************/
/* PB Configuration Structure for the Button using Macros */
const Port_ConfigType BUTTON_PORT_Configuration = {
    .port_num = BUTTON_PORT_NUM,        /* Set port value using macro */
    .pin_num = BUTTON_PIN_NUM,          /* Set pin number value using macro */
    .pin_mode = BUTTON_PIN_MODE,        /* Set Pin Mode using macro */
    .direction = BUTTON_PIN_DIRECTION,  /* Set pin as INPUT using macro */
    .resistor = BUTTON_PIN_RESISTOR,    /* Enable internal pull-up resistor using macro */
    .initial_value = BUTTON_INITIAL_VALUE /* Initial value (Button released, for example) using macro */
};
/*************************************************/

