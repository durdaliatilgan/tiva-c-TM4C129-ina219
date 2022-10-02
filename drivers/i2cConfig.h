#ifndef DRIVERS_I2CCONFIG_H_
#define DRIVERS_I2CCONFIG_H_

/// @brief : this structure created for i2c configuration and initialization
/// it includes necessary information for configuration.

typedef struct
{
    ///base adress (like I2C0_BASE)
    uint32_t i2cBase;

    ///i2c clock frequency
    uint32_t clockFreq;

    /// which gpio pin use for i2c SDA
    uint32_t gpioPin;

    ///SYSCTL_PERIPH_I2C2,SYSCTL_PERIPH_I2C1
    uint32_t peripheral;

    /// port a,b or g (SYSCTL_PERIPH_GPIOG)
    uint32_t port;

    ///GPIO_PORTG_BASE,GPIO_PORTB_BASE
    uint32_t portBase;

    ///GPIO_PG2_I2C2SCL,
    uint32_t portSCL;

    ///SDA oport like GPIO_PG3_I2C2SDA
    uint32_t portSDA;

    ///GPIO_PIN_X FOR SCL Pin
    uint32_t  gpioPinI2CSCL;

}tsI2Cinit;


extern  void initI2C(tsI2Cinit *tsI2c);
extern  uint8_t initI2CChannels(tsI2Cinit *tsI2c,int i2cBase,uint32_t clockFreq);


#endif /* DRIVERS_I2CCONFIG_H_ */
