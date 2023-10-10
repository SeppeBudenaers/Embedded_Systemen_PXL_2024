/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"


/*******************************************************************************
* Macros
*******************************************************************************/
/* Packet positions */
#define PACKET_SOP_POS          (0UL)
#define PACKET_CMD_POS          (1UL)
#define PACKET_EOP_POS          (2UL)
#define PACKET_STATUS_POS       (1UL)


/* I2C slave address to communicate with */
#define I2C_SLAVE_ADDR          (99)

/* I2C bus frequency */
#define I2C_FREQ                (400000UL)

/* I2C slave interrupt priority */
#define I2C_SLAVE_IRQ_PRIORITY  (7u)

/* Command valid status */
#define STATUS_CMD_DONE         (0x00UL)
#define STATUS_CMD_FAIL         (0xFFUL)

/* Packet size */
#define PACKET_SIZE             (1UL)


/*******************************************************************************
* Global Variables
*******************************************************************************/
cyhal_i2c_t sI2C;
uint8_t i2c_write_buffer[PACKET_SIZE] = {0};
uint8_t i2c_read_buffer[PACKET_SIZE] = {0};


/*******************************************************************************
* Function Name: i2c_slave_handle_event
********************************************************************************
* Summary:
* This is a callback function for I2C slave events. If a write event occurs,
* the command packet is verified and executed.
*
* Parameters:
*  callback_arg : extra argument that can be passed to callback
*  event        : I2C event
*
* Return:
*  void
*
*******************************************************************************/
void i2c_slave_handle_event(void *callback_arg, cyhal_i2c_event_t event)
{
    if (0UL == (CYHAL_I2C_SLAVE_ERR_EVENT & event))
    {
        if (0UL != (CYHAL_I2C_SLAVE_WR_CMPLT_EVENT & event))
        {

        }
        }
        if (0UL != (CYHAL_I2C_SLAVE_RD_CMPLT_EVENT & event))
        {
        	cyhal_gpio_write(CYBSP_USER_LED,!i2c_read_buffer[0]);

        	i2c_read_buffer[0] = 0;
        }
    }


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function.
*   1. Initializes the board, retarget-io and led
*   2. Configures the I2C slave to receive packet from the master
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    cyhal_i2c_cfg_t sI2C_cfg;
    /* Initialize the device and board peripherals */
    cybsp_init();
    /* Initialize the retarget-io */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,CY_RETARGET_IO_BAUDRATE);


    /* Initialize user LED */
    printf(">> Configuring user LED..... ");
    cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT,CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* I2C Slave configuration settings */
    sI2C_cfg.is_slave = true;
    sI2C_cfg.address = I2C_SLAVE_ADDR;
    sI2C_cfg.frequencyhal_hz = I2C_FREQ;

    /* Init I2C slave */
    cyhal_i2c_init(&sI2C, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);

    /* Configure I2C slave */
    cyhal_i2c_configure(&sI2C, &sI2C_cfg);


    /* Configure the read buffer on an I2C Slave */
    cyhal_i2c_slave_config_read_buffer(&sI2C, i2c_read_buffer, PACKET_SIZE);
    /* Configure the write buffer on an I2C Slave */
    cyhal_i2c_slave_config_write_buffer(&sI2C, i2c_write_buffer, PACKET_SIZE);
    /* Register an I2C event callback handler */
    cyhal_i2c_register_callback(&sI2C, i2c_slave_handle_event, NULL);
    /* Configure and Enable I2C Interrupt */
    cyhal_i2c_enable_event(&sI2C,
        (cyhal_i2c_event_t)(CYHAL_I2C_SLAVE_WR_CMPLT_EVENT
                           | CYHAL_I2C_SLAVE_RD_CMPLT_EVENT
                           | CYHAL_I2C_SLAVE_ERR_EVENT),
                           I2C_SLAVE_IRQ_PRIORITY, true);

    /* Enable interrupts */
    __enable_irq();

    for (;;)
    {

    }
}
