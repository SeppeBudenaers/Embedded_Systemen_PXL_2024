
/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
int main (void){
    /* Initialize the device and board peripherals */
    cybsp_init();
    /* Enable global interrupts */
    __enable_irq();

    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    cyhal_gpio_init(P0_4, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, true);
    for (;;)
    {
    		if(cyhal_gpio_read(P0_4)== false){
    			Cy_GPIO_Inv(CYBSP_LED9_PORT, CYBSP_LED9_NUM);
    			printf("button pressed\n\r");
    			cyhal_system_delay_ms(500);
    		}
    		else{
    			printf("\n\r");
    		}
    }
}

/* [] END OF FILE */
