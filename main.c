#include <stdint.h>
#include <math.h>
#include <stdbool.h>

#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "sensorlib/i2cm_drv.h"

#include "drivers/ina219.h"
#include "drivers/ina219_hw.h"
#include "drivers/i2cConfig.h"
#include "drivers/i2cConfig.h"

///*****************************************************************************
/// Global instance structure for the I2C master driver.
///*****************************************************************************
tI2CMInstance g_sI2CInst;

///*****************************************************************************
/// Global instance structure for the INA219 sensor driver.
///*****************************************************************************
tINA219 g_sINA219Inst;

///*****************************************************************************
/// Global new data flag to alert main that INA219 data is ready.
///*****************************************************************************
volatile uint_fast8_t g_vui8DataFlag;

///*****************************************************************************
/// The error routine that is called if the driver library encounters an error.
///*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif


///*****************************************************************************
/// INA219 Sensor callback function.  Called at the end of INA219 sensor driver
/// transactions.
///*****************************************************************************
void INA219AppCallback(void* pvCallbackData, uint_fast8_t ui8Status)
{
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        g_vui8DataFlag = 1;
    }
}

///*****************************************************************************
/// Called by the NVIC as a result of I2C0 Interrupt. I2C0 is the I2C connection
/// to the INA219.
///*****************************************************************************
void INA219I2CIntHandler(void)
{
    ///i2cm lib i2c interrunt handler definition.
    I2CMIntHandler(&g_sI2CInst);
}



int main(void)
{

    /// variables declaration for current power and voltage
    float fCurrent,fBusVoltage,fShontVoltage,fPower;

    /// i2c config and init typedef structure
    tsI2Cinit tsI2c;


    /// clock frequency setting for 120MHz clock freq.
    uint32_t ui32SysClock  = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                                                                        SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    ///i2c config and init func
    initI2CChannels(&tsI2c,I2C0_BASE,ui32SysClock);

    ///Enable interrupt
    IntMasterEnable();

    /// initialize i2c
    I2CMInit(&g_sI2CInst, I2C0_BASE, INT_I2C0, 0, 0, ui32SysClock);

    g_vui8DataFlag = 0;

    ///ina219 calibration func.
    INA219Calibrate(&g_sINA219Inst, &g_sI2CInst, INA219_ADDRESS,
                    INA219AppCallback,&g_sINA219Inst, INA219_CALIB_32V_2A);

    ///wait for i2c sending and receiving ended successfully. and flag to zero
    while(g_vui8DataFlag == 0);  g_vui8DataFlag = 0;

    ///ina219 configuration func.
    INA219Configuration(&g_sINA219Inst, &g_sI2CInst, INA219_ADDRESS,
                        INA219AppCallback, &g_sINA219Inst, INA219_CALIB_32V_2A);


    ///wait for i2c sending and receiving ended successfully. and flag to zero
    while(g_vui8DataFlag == 0); g_vui8DataFlag = 0;


    while(1)
    {

        /// read 16 bit current value
        INA219ReadCurrentRaw(&g_sINA219Inst, INA219AppCallback,
                             &g_sINA219Inst,INA219_REG_CURRENT);

        ///wait for i2c sending and receiving ended successfully.
        while(g_vui8DataFlag == 0); g_vui8DataFlag = 0;


        /// calculation float current value
        INA219CalculateCurrent(&g_sINA219Inst, &fCurrent);

        ///wait for i2c sending and receiving ended successfully.
        while(g_vui8DataFlag == 0); g_vui8DataFlag = 0;

        /// read 16 bit power calue
        INA219ReadPowerRaw(&g_sINA219Inst, INA219AppCallback,
                           &g_sINA219Inst,INA219_REG_POWER);

        ///wait for i2c sending and receiving ended successfully. and flag to zero
        while(g_vui8DataFlag == 0); g_vui8DataFlag = 0;

        /// calculation float power value
        INA219CalculatePower(&g_sINA219Inst, &fPower);

        ///wait for i2c sending and receiving ended successfully. and flag to zero
        while(g_vui8DataFlag == 0); g_vui8DataFlag = 0;


        INA219ReadBusVoltage(&g_sINA219Inst, INA219AppCallback, &g_sINA219Inst);

        /// calculation float power value
        INA219CalculateBusVoltage(&g_sINA219Inst, &fBusVoltage);

        ///wait for i2c sending and receiving ended successfully. and flag to zero
        while(g_vui8DataFlag == 0); g_vui8DataFlag = 0;

        INA219ReadShontVoltage(&g_sINA219Inst, INA219AppCallback, &g_sINA219Inst);

        /// calculation float power value
        INA219CalculateShontVoltage(&g_sINA219Inst, &fShontVoltage);

        ///wait for i2c sending and receiving ended successfully. and flag to zero
        while(g_vui8DataFlag == 0); g_vui8DataFlag = 0;
    }
}


