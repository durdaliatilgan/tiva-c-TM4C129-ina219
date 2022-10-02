#include <stdint.h>
#include <stdbool.h>
#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "drivers/i2cConfig.h"



/// @brief   i2c channel configuration func.
///
/// @param tsI2c: i2c typedef structure address
/// @param i2cBase: i2c BASE name
/// @param clockFreq: System clock frequency
/// @return 0 :error, 1: success
uint8_t initI2CChannels(tsI2Cinit *tsI2c, int i2cBase,uint32_t clockFreq){

    switch(i2cBase)
    {

        default:
        {
            return 0;
        }
        case I2C0_BASE:
        {
            tsI2c->i2cBase=I2C0_BASE;
            tsI2c->peripheral=SYSCTL_PERIPH_I2C0;
            tsI2c->gpioPin=GPIO_PIN_3;
            tsI2c->gpioPinI2CSCL=GPIO_PIN_2;
            tsI2c->port=SYSCTL_PERIPH_GPIOB;
            tsI2c->portBase=GPIO_PORTB_BASE;
            tsI2c->portSCL=GPIO_PB2_I2C0SCL;
            tsI2c->portSDA=GPIO_PB3_I2C0SDA;
            tsI2c->clockFreq=clockFreq;
            initI2C(tsI2c);
            break;
        }
        case I2C1_BASE:
        {
            tsI2c->i2cBase=i2cBase;
            tsI2c->peripheral=SYSCTL_PERIPH_I2C1;
            tsI2c->gpioPin=GPIO_PIN_1;
            tsI2c->gpioPinI2CSCL=GPIO_PIN_0;
            tsI2c->port=SYSCTL_PERIPH_GPIOG;
            tsI2c->portBase=GPIO_PORTG_BASE;
            tsI2c->portSCL=GPIO_PG0_I2C1SCL;
            tsI2c->portSDA=GPIO_PG1_I2C1SDA;
            tsI2c->clockFreq=clockFreq;
            initI2C(tsI2c);
            break;
        }
        case I2C2_BASE:
        {
            tsI2c->i2cBase=i2cBase;
            tsI2c->peripheral=SYSCTL_PERIPH_I2C2;
            tsI2c->gpioPin=GPIO_PIN_3;
            tsI2c->gpioPinI2CSCL=GPIO_PIN_2;
            tsI2c->port=SYSCTL_PERIPH_GPIOG;
            tsI2c->portBase=GPIO_PORTG_BASE;
            tsI2c->portSCL=GPIO_PG2_I2C2SCL;
            tsI2c->portSDA=GPIO_PG3_I2C2SDA;
            tsI2c->clockFreq=clockFreq;
            initI2C(tsI2c);
            break;

        }
        case I2C3_BASE:
        {
            tsI2c->i2cBase=i2cBase;
            tsI2c->peripheral=SYSCTL_PERIPH_I2C3;
            tsI2c->gpioPin=GPIO_PIN_5;
            tsI2c->gpioPinI2CSCL=GPIO_PIN_4;
            tsI2c->port=SYSCTL_PERIPH_GPIOG;
            tsI2c->portBase=GPIO_PORTG_BASE;
            tsI2c->portSCL=GPIO_PG4_I2C3SCL;
            tsI2c->portSDA=GPIO_PG5_I2C3SDA;
            tsI2c->clockFreq=clockFreq;
            initI2C(tsI2c);
            break;
        }
        case I2C4_BASE:
        {
            tsI2c->i2cBase=i2cBase;
            tsI2c->peripheral=SYSCTL_PERIPH_I2C4;
            tsI2c->gpioPin=GPIO_PIN_7;
            tsI2c->gpioPinI2CSCL=GPIO_PIN_6;
            tsI2c->port=SYSCTL_PERIPH_GPIOG;
            tsI2c->portBase=GPIO_PORTG_BASE;
            tsI2c->portSCL=GPIO_PG6_I2C4SCL;
            tsI2c->portSDA=GPIO_PG7_I2C4SDA;
            tsI2c->clockFreq=clockFreq;
            initI2C(tsI2c);
            break;
        }
        case I2C5_BASE:
        {
            tsI2c->i2cBase=i2cBase;
            tsI2c->peripheral=SYSCTL_PERIPH_I2C5;
            tsI2c->gpioPin=GPIO_PIN_1;
            tsI2c->gpioPinI2CSCL=GPIO_PIN_0;
            tsI2c->port=SYSCTL_PERIPH_GPIOB;
            tsI2c->portBase=GPIO_PORTB_BASE;
            tsI2c->portSCL=GPIO_PB0_I2C5SCL;
            tsI2c->portSDA=GPIO_PB1_I2C5SDA;
            tsI2c->clockFreq=clockFreq;
            initI2C(tsI2c);
            break;

        }
        case I2C6_BASE:
        {
            tsI2c->i2cBase=i2cBase;
            tsI2c->peripheral=SYSCTL_PERIPH_I2C6;
            tsI2c->gpioPin=GPIO_PIN_7;
            tsI2c->gpioPinI2CSCL=GPIO_PIN_6;
            tsI2c->port=SYSCTL_PERIPH_GPIOA;
            tsI2c->portBase=GPIO_PORTA_BASE;
            tsI2c->portSCL=GPIO_PA6_I2C6SCL;
            tsI2c->portSDA=GPIO_PA7_I2C6SDA;
            tsI2c->clockFreq=clockFreq;
            initI2C(tsI2c);
            break;
        }
    }
    return 1;
}

/// @brief selected i2c port initialize by initI2C func.
///
/// @param tsI2c: included  i2c port information structure

void initI2C(tsI2Cinit *tsI2c)
{
    ///enable I2C module
    SysCtlPeripheralEnable(tsI2c->peripheral);

    ///reset I2C module
    SysCtlPeripheralReset(tsI2c->peripheral);

    ///enable GPIO peripheral that contains I2C
    SysCtlPeripheralEnable(tsI2c->port);

    ///Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(tsI2c->portSCL);
    GPIOPinConfigure(tsI2c->portSDA);

    /// Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(tsI2c->portBase, tsI2c->gpioPinI2CSCL);
    GPIOPinTypeI2C(tsI2c->portBase, tsI2c->gpioPin);

}
