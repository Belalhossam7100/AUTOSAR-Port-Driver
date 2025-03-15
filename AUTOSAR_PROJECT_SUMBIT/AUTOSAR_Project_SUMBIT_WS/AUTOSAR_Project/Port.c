/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mohamed Tarek
 ******************************************************************************/

#include "Port.h"
#include "tm4c123gh6pm_registers.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Port Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Det.h does not match the expected version"
#endif

#endif

STATIC const volatile uint32  * Port_Ptr = NULL_PTR; /* point to the required Port Registers base address */
STATIC const volatile Port_ConfigType  * Global_config_ptr = NULL_PTR; /* point to the required Port Registers base address */

STATIC uint8 PORT_Status = PORT_NOT_INITIALIZED;

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
void Port_Init(const Port_ConfigType* ConfigPtr)
{

 #if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if the input configuration pointer is not a NULL_PTR */
    if (NULL_PTR == ConfigPtr)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_Init_SID,
             PORT_E_PARAM_CONFIG);
    }
    else
 #endif
    {
        /* Global Pointer to configuration set */
        Global_config_ptr = ConfigPtr;
        ////////*********Set the pointer to the required Port******************************//
        switch(ConfigPtr->port_num)
        {
            case  PORT_A: Port_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                     break;
            case  PORT_B: Port_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                     break;
            case  PORT_C: Port_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                     break;
            case  PORT_D: Port_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                     break;
            case  PORT_E: Port_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                     break;
            case  PORT_F: Port_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                     break;
        }

        if ( ((ConfigPtr->port_num == PORT_D) && (ConfigPtr->pin_num == PIN_7)) ||
             ((ConfigPtr->port_num == PORT_F) && (ConfigPtr->pin_num == PIN_0)) )  /* PD7 or PF0 */
        {
        *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_COMMIT_REG_OFFSET) ,ConfigPtr->pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
        }
        else if ((ConfigPtr->port_num == PORT_C) && (ConfigPtr->pin_num <= PIN_3)) /* PC0 to PC3 */
        {
        /* Do Nothing ...  this is the JTAG pins */
        }
        else
        {
        /* Do Nothing ... No need to unlock the commit register for this pin */
        }
       ////////**********Enable clock for PORT******************************//
       /* Enable clock for PORT and wait for clock to start */
       switch(ConfigPtr->port_num)
       {
            case PORT_A: SYSCTL_RCGCGPIO_REG |= 0x01;  // Activate clock for PORTA
                    while (!(SYSCTL_PRGPIO_REG & 0x01));  // Wait until PORTA is ready
                    break;

            case PORT_B: SYSCTL_RCGCGPIO_REG |= 0x02;  // Activate clock for PORTB
                    while (!(SYSCTL_PRGPIO_REG & 0x02));  // Wait until PORTB is ready
                    break;

            case PORT_C: SYSCTL_RCGCGPIO_REG |= 0x04;  // Activate clock for PORTC
                    while (!(SYSCTL_PRGPIO_REG & 0x04));  // Wait until PORTC is ready
                    break;

            case PORT_D: SYSCTL_RCGCGPIO_REG |= 0x08;  // Activate clock for PORTD
                    while (!(SYSCTL_PRGPIO_REG & 0x08));  // Wait until PORTD is ready
                    break;

            case PORT_E: SYSCTL_RCGCGPIO_REG |= 0x10;  // Activate clock for PORTE
                    while (!(SYSCTL_PRGPIO_REG & 0x10));  // Wait until PORTE is ready
                    break;

            case PORT_F: SYSCTL_RCGCGPIO_REG |= 0x20;  // Activate clock for PORTF
                    while (!(SYSCTL_PRGPIO_REG & 0x20));  // Wait until PORTF is ready
                    break;
            default:
                    // Invalid port
                    break;
       }
   PORT_Status = PORT_INITIALIZED;
   if ((Global_config_ptr->port_num == PORT_C) && (Global_config_ptr->pin_num <= PIN_3)) /* PC0 to PC3 */
   {
       /* Do Nothing ...  this is the JTAG pins */
   }
   else
   {
     /////////**********Sets the port pin direction.**********/////////
     Port_SetPinDirection(ConfigPtr->pin_num, ConfigPtr->direction );
     /////////**********Sets the port pin Mode.**********/////////
     Port_SetPinMode((ConfigPtr->pin_num),(ConfigPtr->pin_mode));
   }

}
}
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
#if(PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection(
        Port_PinType Pin,
        Port_PinDirectionType Direction
){
#if (PORT_DEV_ERROR_DETECT == STD_ON)
  /* Check if the Driver is initialized before using this function */
  if (PORT_NOT_INITIALIZED == PORT_Status)
  {
      Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                      Port_SetPinDirection_SID, PORT_E_UNINIT);
  }
  else
  {
      /* No Action Required */
  }
#endif
    {
    if((Global_config_ptr->direction) == PORT_PIN_OUT)
    {
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIR_REG_OFFSET) , (Global_config_ptr->pin_num));               /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
        if(Global_config_ptr->initial_value == STD_HIGH)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DATA_REG_OFFSET) , (Global_config_ptr->pin_num));          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
        }
        else
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DATA_REG_OFFSET) , (Global_config_ptr->pin_num));        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
        }
    }

    else if(Global_config_ptr->direction == PORT_PIN_IN)
    {
      CLEAR_BIT((*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIR_REG_OFFSET)) , (Global_config_ptr->pin_num));             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */

        if(Global_config_ptr->resistor == PULL_UP)
        {
            SET_BIT((*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_PULL_UP_REG_OFFSET)) , (Global_config_ptr->pin_num));       /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
        }
        else if(Global_config_ptr->resistor == PULL_DOWN)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_PULL_DOWN_REG_OFFSET) , (Global_config_ptr->pin_num));     /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
        }
        else if(Global_config_ptr->resistor == OFF)
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_PULL_UP_REG_OFFSET) , (Global_config_ptr->pin_num));     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_PULL_DOWN_REG_OFFSET) , (Global_config_ptr->pin_num));   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
        }
    }
    else
    {
        /* Do Nothing */
    }
 }
}
#endif
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
void Port_RefreshPortDirection( void ){
    #if (PORT_DEV_ERROR_DETECT == STD_ON)
       /* Check if the input configuration pointer is not a NULL_PTR */
       if (NULL_PTR == Global_config_ptr)
       {
           Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_RefreshPortDirection_SID,
                           PORT_E_DIRECTION_UNCHANGEABLE);
       }
       else
    #endif
       {
               /* Set the pin direction */
               Port_SetPinDirection(Global_config_ptr->pin_num, Global_config_ptr->direction);
       }

}
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
void Port_GetVersionInfo( Std_VersionInfoType* versioninfo ){
  #if (PORT_DEV_ERROR_DETECT == STD_ON)
   /* Check if the input configuration pointer is not a NULL_PTR */
  if (NULL_PTR == Global_config_ptr)
  {
       Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_GetVersionInfo_SID,
                       PORT_E_PARAM_POINTER);
  }
   else
  #endif
  {
       versioninfo->vendorID=PORT_VENDOR_ID;
       versioninfo->moduleID=PORT_MODULE_ID;
       versioninfo->sw_major_version=PORT_SW_MAJOR_VERSION;
       versioninfo->sw_minor_version=PORT_SW_MINOR_VERSION;
       versioninfo->sw_patch_version=PORT_SW_PATCH_VERSION;
  }

}
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
void Port_SetPinMode( Port_PinType Pin,
                      Port_PinModeType Mode)
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
 /* Check if the input configuration pointer is not a NULL_PTR */
 if (PORT_PIN_MODE_CHANGEBLE == STD_OFF)
 {
     Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinMode_SID, PORT_E_MODE_UNCHANGEABLE);
 }
 else
  /* Check if the Driver is initialized before using this function */
 if (PORT_NOT_INITIALIZED == PORT_Status)
  {
      Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                      Port_SetPinDirection_SID, PORT_E_UNINIT);
  }
 else
  {
      /* No Action Required */
  }
#endif
  {
    switch(Global_config_ptr->pin_mode)
    {
        case PORT_PIN_MODE_GPIO:
            /* Setup the pin mode as GPIO */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , (Global_config_ptr->pin_num));      /* Disable  analog function */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ALT_FUNC_REG_OFFSET) , (Global_config_ptr->pin_num));              /* Disable  alternate function */
            *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Global_config_ptr->pin_num * 4));     /* Clear the PMCx bits for this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , (Global_config_ptr->pin_num));         /* Enable  digital function */
        break;
        case PORT_PIN_MODE_ADC:
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , (Global_config_ptr->pin_num));      /* Enable analog function */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ALT_FUNC_REG_OFFSET) , (Global_config_ptr->pin_num));             /* Enable alternate function */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , (Global_config_ptr->pin_num));        /* Disable digital function */
        break;
        case PORT_PIN_MODE_UART:
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , (Global_config_ptr->pin_num));      /* Disable  analog function */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ALT_FUNC_REG_OFFSET) , (Global_config_ptr->pin_num));              /* Enable  alternate function */
            *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) & ~(0x0000000F << ((Global_config_ptr->pin_num) * 4)) | (0x00000001 << ((Global_config_ptr->pin_num) * 4));     /* 0x1 in the PMCx bits for this pin to use the UART Mode */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , (Global_config_ptr->pin_num));         /* Enable  digital function */
        break;
        case PORT_PIN_MODE_I2C:
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , (Global_config_ptr->pin_num));      /* Disable  analog function */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ALT_FUNC_REG_OFFSET) , (Global_config_ptr->pin_num));              /* Enable  alternate function */
            *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) & ~(0x0000000F << ((Global_config_ptr->pin_num) * 4)) | (0x00000003 << ((Global_config_ptr->pin_num) * 4));     /*  0x3 in the PMCx bits for this pin to use the I2C Mode */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , (Global_config_ptr->pin_num));         /* Enable  digital function */
        break;
        case PORT_PIN_MODE_SSI:
           CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , (Global_config_ptr->pin_num));      /* Disable  analog function */
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ALT_FUNC_REG_OFFSET) , (Global_config_ptr->pin_num));              /* Enable  alternate function */
           *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) & ~(0x0000000F << ((Global_config_ptr->pin_num) * 4)) | (0x00000002 << ((Global_config_ptr->pin_num) * 4));     /*  0x2 in the PMCx bits for this pin to use the SSI Mode */
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , (Global_config_ptr->pin_num));         /* Enable  digital function */
        break;
        case PORT_PIN_MODE_CAN:
            /* Configure pin for CAN */
        break;
        case PORT_PIN_MODE_PWM:
            /* Configure pin for PWM */
        break;
        case PORT_PIN_MODE_GPT:
            /* Configure pin for General Purpose Timer */
        break;
        case PORT_PIN_MODE_WDG:
            /* Configure pin for WATCH DOG TIMER Timer */
        break;
        case PORT_PIN_MODE_USB:
            /* Configure pin for WATCH DOG TIMER Timer */
        break;
        default:
                /* Handle invalid pin mode*/
        break;
     }

  }
}
//////////////////////////////////////////////////////////////////////////////////////**********************************************************************************************************************/
