/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mohamed Tarek
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H

/* Id for the company in the AUTOSAR
 * for example Mohamed Tarek's ID = 1000 :) */
#define PORT_VENDOR_ID    (1000U)

/* Port Module Id */
#define PORT_MODULE_ID    (121U)  // Example Module ID for the Port Module

/* Port Instance Id */
#define PORT_INSTANCE_ID  (0U)

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)

/*
 * Macros for Port Status
 */
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)

/* Standard AUTOSAR types */
#include "Std_Types.h"

/* AUTOSAR checking between Std Types and Port Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION) \
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION) \
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif

/* Port Pre-Compile Configuration Header file */
#include "Port_Cfg.h"

/* AUTOSAR Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION) \
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION) \
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Port_Cfg.h does not match the expected version"
#endif

/* Software Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION) \
 ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION) \
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of Port_Cfg.h does not match the expected version"
#endif

/* Non AUTOSAR files */
#include "Common_Macros.h"

/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/
/* Service ID for the Port_Init function */
#define Port_Init_SID           (uint8)0x00

/* Service ID for the Port_SetPinDirection function */
#define Port_SetPinDirection_SID           (uint8)0x01

/* Service ID for the Port_RefreshPortDirection function */
#define Port_RefreshPortDirection_SID           (uint8)0x02

/* Service ID for the Port_GetVersionInfo function */
#define Port_GetVersionInfo_SID           (uint8)0x03

/* Service ID for the Port_SetPinMode function */
#define Port_SetPinMode_SID           (uint8)0x04

/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/
/* DET code to report Invalid Port Pin ID requested */
#define PORT_E_PARAM_PIN                (uint8)0x0A

/* DET code to report that Port Pin is not configured as changeable */
#define PORT_E_DIRECTION_UNCHANGEABLE   (uint8)0x0B

/* DET code to report API Port_Init service called with wrong parameter */
#define PORT_E_PARAM_CONFIG             (uint8)0x0C

/* DET code to report API Port_SetPinMode service called with invalid mode */
#define PORT_E_PARAM_INVALID_MODE       (uint8)0x0D

/* DET code to report API Port_SetPinMode service called when mode is unchangeable */
#define PORT_E_MODE_UNCHANGEABLE        (uint8)0x0E

/* DET code to report API service called without module initialization */
#define PORT_E_UNINIT                   (uint8)0x0F

/* DET code to report APIs called with a Null Pointer */
#define PORT_E_PARAM_POINTER            (uint8)0x10

/*******************************************************************************
 *                              Module Definitions                             *
 *******************************************************************************/

/* GPIO Registers base addresses */
#define GPIO_PORTA_BASE_ADDRESS           0x40004000
#define GPIO_PORTB_BASE_ADDRESS           0x40005000
#define GPIO_PORTC_BASE_ADDRESS           0x40006000
#define GPIO_PORTD_BASE_ADDRESS           0x40007000
#define GPIO_PORTE_BASE_ADDRESS           0x40024000
#define GPIO_PORTF_BASE_ADDRESS           0x40025000

/* GPIO Registers offset addresses */
#define PORT_DATA_REG_OFFSET              0x3FC
#define PORT_DIR_REG_OFFSET               0x400
#define PORT_ALT_FUNC_REG_OFFSET          0x420
#define PORT_PULL_UP_REG_OFFSET           0x510
#define PORT_PULL_DOWN_REG_OFFSET         0x514
#define PORT_DIGITAL_ENABLE_REG_OFFSET    0x51C
#define PORT_LOCK_REG_OFFSET              0x520
#define PORT_COMMIT_REG_OFFSET            0x524
#define PORT_ANALOG_MODE_SEL_REG_OFFSET   0x528
#define PORT_CTL_REG_OFFSET               0x52C

/* Enumeration for Port Numbers */
typedef enum {
    PORT_A,   // Port A - 0
    PORT_B,   // Port B - 1
    PORT_C,   // Port C - 2
    PORT_D,   // Port D - 3
    PORT_E,   // Port E - 4
    PORT_F    // Port F - 5
} Port_Num;

/* Enumeration for Pin Numbers */
typedef enum {
    PIN_0,
    PIN_1,
    PIN_2,
    PIN_3,
    PIN_4,
    PIN_5,
    PIN_6,
    PIN_7
} Pin_Num;

/* Enum for PortPinInitialMode - Port pin mode from the mode list for use with the Port_Init() function. */
typedef enum {
    PORT_PIN_MODE_GPIO,         // Port Pin configured for GPIO (General Purpose Input/Output)
    PORT_PIN_MODE_GPT,          // Port Pin configured for General Purpose Timer
    PORT_PIN_MODE_WDG,          // Port Pin configured for Watchdog Timer
    PORT_PIN_MODE_ADC,          // Port Pin used by ADC (Analog-to-Digital Converter)
    PORT_PIN_MODE_UART,         // Port Pin used for UART (Universal Asynchronous Receiver/Transmitter)
    PORT_PIN_MODE_I2C,          // Port Pin used for I2C (Inter-Integrated Circuit)
    PORT_PIN_MODE_SSI,          // Port Pin used for SSI (Synchronous Serial Interface)
    PORT_PIN_MODE_CAN,          // Port Pin used for CAN (Controller Area Network)
    PORT_PIN_MODE_USB,          // Port Pin used for USB (Universal Serial Bus)
    PORT_PIN_MODE_PWM          // Port Pin used by PWM (Pulse Width Modulation)
} PORT_PIN_MODE;


/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/
/* Description: Different port pin modes.*/
typedef uint8 Port_PinModeType;

/* Description: Data type for the symbolic name of a port pin.*/
typedef uint8 Port_PinType;

/* Description: Enum to hold PIN direction */
typedef enum
{
    PORT_PIN_IN,PORT_PIN_OUT
}Port_PinDirectionType;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
    OFF,PULL_UP,PULL_DOWN
}Port_InternalResistor;

/* Description: Structure to configure each individual PIN:
 *      1. the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
 *      2. the number of the pin in the PORT.
 *      3.pin_mode --> DIO,ADC,SPI,....
 *      4. the direction of pin --> INPUT or OUTPUT
 *      5. the internal resistor --> Disable, Pull up or Pull down
 */
typedef struct {
    uint8 port_num;
    uint8 pin_num;
    Port_PinModeType  pin_mode;/*DIO,ADC,SPI,....*/
    Port_PinDirectionType direction;
    Port_InternalResistor resistor;
    uint8 initial_value;
} Port_ConfigType;



/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/
//////////////////////////////////////////////////////////////////////////////////////**********************************************************************************************************************/
/************************************************************************************
* Service Name: Port_Init
* Sync/Async: Synchronous
* Reentrancy: Non-reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description:Initializes the Port Driver module.
************************************************************************************/
void Port_Init(const Port_ConfigType* ConfigPtr);
//////////////////////////////////////////////////////////////////////////////////////**********************************************************************************************************************/
/************************************************************************************
* Service Name: Port_SetPinDirection
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): Pin - Port Pin ID number
                   Direction - Port Pin Direction
* Parameters (out): None
* Return value: None
* Description:Sets the port pin direction.
************************************************************************************/
void Port_SetPinDirection( Port_PinType Pin, Port_PinDirectionType Direction );
//////////////////////////////////////////////////////////////////////////////////////**********************************************************************************************************************/
/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Sync/Async: Synchronous
* Reentrancy: Non-reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description:Initializes Refreshes port direction.
************************************************************************************/
void Port_RefreshPortDirection(void);
//////////////////////////////////////////////////////////////////////////////////////**********************************************************************************************************************/
/************************************************************************************
* Service Name: Port_GetVersionInfo
* Sync/Async: Synchronous
* Reentrancy: Non-reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): versioninfo - Pointer to where to store the version information of this module.
* Return value: None
* Description: Returns the version information of this module.
************************************************************************************/
void Port_GetVersionInfo( Std_VersionInfoType* versioninfo );
//////////////////////////////////////////////////////////////////////////////////////**********************************************************************************************************************/
/************************************************************************************
* Service Name: Port_SetPinMode
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): Pin - Port Pin ID number
                   Mode - New Port Pin mode to be set on port pin.
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description:Sets the port pin mode.
************************************************************************************/
void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode );
//////////////////////////////////////////////////////////////////////////////////////**********************************************************************************************************************/

/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/

/* Extern PB structures to be used by Port_Init */
extern const Port_ConfigType LED_PORT_Configuration;
/* Extern PB structures to be used by Port_Init*/
extern const Port_ConfigType BUTTON_PORT_Configuration;
//////////////////////////////////////////////////////////////////////////////////////**********************************************************************************************************************/

#endif /* PORT_H */
