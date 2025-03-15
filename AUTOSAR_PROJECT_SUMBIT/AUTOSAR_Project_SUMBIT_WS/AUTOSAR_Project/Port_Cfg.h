/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Mohamed Tarek
 ******************************************************************************/
#ifndef PORT_CFG_H_
#define PORT_CFG_H_

/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (3U)

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT                (STD_ON)

/* Pre-compile option for presence of Port_SetPinDirection API */
#define PORT_SET_PIN_DIRECTION_API           (STD_ON)

/* Pre-compile option for presence of PORT_SET_PIN_MODE API */
#define PORT_SET_PIN_MODE_API           (STD_ON)

/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API                (STD_ON)

/* Port Configured PortPinModeChangeable */
#define PORT_PIN_MODE_CHANGEBLE     (STD_ON)

/* Number of the configured pin */
#define PORT_CONFIGURED_PINS              (38U) // total of 43 General-Purpose Input/Output (GPIO) pins. (5 of them are JTAG pins)

/* Configuration Macros for LED */
#define LED_PORT_NUM        PORT_F
#define LED_PIN_NUM         1
#define LED_PIN_MODE        PORT_PIN_MODE_GPIO
#define LED_PIN_DIRECTION   PORT_PIN_OUT
#define LED_PIN_RESISTOR    OFF
#define LED_INITIAL_VALUE   STD_LOW
/*********************************/

/* Configuration Macros for Button */
#define BUTTON_PORT_NUM         PORT_F
#define BUTTON_PIN_NUM          4
#define BUTTON_PIN_MODE         PORT_PIN_MODE_GPIO
#define BUTTON_PIN_DIRECTION    PORT_PIN_IN
#define BUTTON_PIN_RESISTOR     PULL_UP
#define BUTTON_INITIAL_VALUE    STD_LOW  /*  1 represents BUTTON_RELEASED */

#endif /* PORT_CFG_H_ */
