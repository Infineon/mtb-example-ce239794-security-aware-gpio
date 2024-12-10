/*****************************************************************************
* File Name        : main.c
*
* Description      : This source file contains the main routine for non-secure
*                    application in the CM33 CPU
*
* Related Document : See README.md
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/


/******************************************************************************
 * Header Files
 *****************************************************************************/

#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"


/******************************************************************************
 * Macros
 *****************************************************************************/

/* Non-secure Interrupt priority */
#define NS_ISR_PRIORITY     5u


/******************************************************************************
 * Global Variables
 *****************************************************************************/
/* For the Retarget -IO (Debug UART) usage */
static cy_stc_scb_uart_context_t    DEBUG_UART_context;           /** UART context */
static mtb_hal_uart_t               DEBUG_UART_hal_obj;           /** Debug UART HAL object  */


/******************************************************************************
 * Function Prototypes
 *****************************************************************************/

/**
 * @brief NS GPIO Handler.
 *
 * This is the NS GPIO interrupt handler. The handler toggles the NS LED.
 * 
 * @return Void.
 */
void ns_gpio_interrupt_handler(void);


/******************************************************************************
 * Function Definitions
 *****************************************************************************/

void ns_gpio_interrupt_handler(void)
{
    Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN2_PORT, CYBSP_USER_BTN2_PIN);
    Cy_GPIO_Inv(CYBSP_USER_LED2_PORT, CYBSP_USER_LED2_PIN);
}


/**
 * @brief Main function of M33 Non-secure project
 *
 * This is the non-secure main function for M33 CPU. It configures a NS LED 
 * and NS button. The NS button generates a NS IRQ which toggles NS LED.
 *
 * @return Should not return
 *
 */
int main(void)
{
    cy_rslt_t result = 0xFFFFFFFF;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Debug UART init */
    result = (cy_rslt_t)Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);

    /* UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    Cy_SCB_UART_Enable(DEBUG_UART_HW);

    /* Setup the HAL UART */
    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, &DEBUG_UART_hal_config, &DEBUG_UART_context, NULL);

    /* HAL UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);

    /* HAL retarget_io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }


    /* Set non-secure interrupt */
    NVIC_SetVector(CYBSP_USER_BTN2_IRQ, (uint32_t)ns_gpio_interrupt_handler);

    NVIC_SetPriority(CYBSP_USER_BTN2_IRQ, NS_ISR_PRIORITY);

    NVIC_EnableIRQ(CYBSP_USER_BTN2_IRQ);

    printf("\x1b[2J\x1b[;H");
    printf("********* PSOC Control MCU: Security Aware GPIO *********\r\n\n");

    printf("Press USER_BTN1 to toggle Secure LED\r\n\n");

    printf("Press USER_BTN2 to toggle Non-secure LED\r\n\n");

    for (;;)
    {

    }
}
/* [] END OF FILE */

