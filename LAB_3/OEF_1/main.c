/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"


/*******************************************************************************
* Macros
*******************************************************************************/
/* Delay of 1000ms between commands */
#define CMD_TO_CMD_DELAY        (1000UL)

/* Packet positions */
#define PACKET_CMD_POS          (0UL)

/* I2C slave address to communicate with */
#define I2C_SLAVE_ADDR          (63)

/* I2C bus frequency */
#define I2C_FREQ                (400000UL)

/* Packet size */
#define PACKET_SIZE             (1UL)

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function.
*   1. I2C Master sends command packet to the slave
*   2. I2C Master reads the response packet to generate the next command
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
    cyhal_i2c_t mI2C;
    cyhal_i2c_cfg_t mI2C_cfg;
    uint8_t cmd = 0b00000000;
    uint8_t buffer[PACKET_SIZE];

    /* Initialize the device and board peripherals */
    cybsp_init();


    /* Initialize the retarget-io */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,CY_RETARGET_IO_BAUDRATE);

    /* I2C Master configuration settings */
    mI2C_cfg.is_slave = false;
    mI2C_cfg.address = 0;
    mI2C_cfg.frequencyhal_hz = I2C_FREQ;

    /* Init I2C master */
    cyhal_i2c_init(&mI2C, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);

    /* Configure I2C Master */
    cyhal_i2c_configure(&mI2C, &mI2C_cfg);

    /* Enable interrupts */
    __enable_irq();

    for (;;)
    {
        /* Create packet to be sent to slave */
        buffer[PACKET_CMD_POS] = cmd;//cmd is your packet

        /* Send packet with command to the slave */
        cyhal_i2c_master_write(&mI2C, I2C_SLAVE_ADDR,buffer, PACKET_SIZE, 0, false);

        /* Give delay between commands */
         cyhal_system_delay_ms(CMD_TO_CMD_DELAY);
    }
}
