#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"

#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

#include "sensorlib/i2cm_drv.h"

#include "drivers/ina219.h"
#include "drivers/ina219_hw.h"




//*****************************************************************************
//
// Global instance structure for the I2C master driver.
//
//*****************************************************************************
tI2CMInstance g_sI2CInst;

//*****************************************************************************
//
// Global instance structure for the INA219 sensor driver.
//
//*****************************************************************************
tINA219 g_sINA219Inst;

//*****************************************************************************
//
// Global new data flag to alert main that INA219 data is ready.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8DataFlag;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif




void initI2C0(void)
{
   SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

   //reset I2C module
   SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

   //enable GPIO peripheral that contains I2C
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

   // Configure the pin muxing for I2C0 functions on port B2 and B3.
   GPIOPinConfigure(GPIO_PB2_I2C0SCL);
   GPIOPinConfigure(GPIO_PB3_I2C0SDA);

   // Select the I2C function for these pins.
   GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
   GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

   // Enable and initialize the I2C0 master module.  Use the system clock for
   // the I2C0 module.  The last parameter sets the I2C data transfer rate.
   // If false the data rate is set to 100kbps and if true the data rate will
   // be set to 400kbps.
   //I2CMasterInitExpClk(I2C0_BASE, 25'000'000, true);
   I2CMasterInitExpClk(I2C0_BASE, 40000000, false);
   //clear I2C FIFOs
  // HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}

//*****************************************************************************
//
// BMP180 Sensor callback function.  Called at the end of INA219 sensor driver
// transactions. This is called from I2C interrupt context. Therefore, we just
// set a flag and let main do the bulk of the computations and display.
//
//*****************************************************************************
void INA219AppCallback(void* pvCallbackData, uint_fast8_t ui8Status)
{
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        g_vui8DataFlag = 1;
    }
}

//*****************************************************************************
//
// Called by the NVIC as a result of I2C3 Interrupt. I2C3 is the I2C connection
// to the INA219.
//
//*****************************************************************************
void
INA219I2CIntHandler(void)
{
    //
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    //
    I2CMIntHandler(&g_sI2CInst);
}

float fCurrent,fBusVoltage,fShontVoltage,fPower;

//*****************************************************************************
//
// Main 'C' Language entry point.
//
//*****************************************************************************
int
main(void)
{

    uint32_t ui32SysClock;
    char pcBuf[15];

    //
    // Setup the system clock to run at 40 Mhz from PLL with crystal reference
    //
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                           SYSCTL_OSC_MAIN |
                                           SYSCTL_USE_PLL |
                                           SYSCTL_CFG_VCO_480), 40000000);



    initI2C0();

    //
    // Enable interrupts to the processor.
    //
    IntMasterEnable();

    //
    // Initialize I2C3 peripheral.
    //
    I2CMInit(&g_sI2CInst, I2C0_BASE, INT_I2C0, 0, 0,
             ui32SysClock);

    //
    // Initialize the INA219
    //
    INA219Calibrate(&g_sINA219Inst, &g_sI2CInst, INA219_ADDRESS, INA219AppCallback, &g_sINA219Inst, INA219_CALIB_32V_2A);
    INA219Configuration(&g_sINA219Inst, &g_sI2CInst, INA219_ADDRESS,INA219AppCallback, &g_sINA219Inst, INA219_CALIB_32V_2A);

    //
    // Wait for initialization callback to indicate reset request is complete.
    //
    while(g_vui8DataFlag == 0);

    //
    // Reset the data ready flag
    //
    g_vui8DataFlag = 0;


    //
    // Begin the data collection and printing.  Loop Forever.
    //
    while(1)
    {
        //
        // Read the data from the INA219 over I2C.  This command starts a
        // temperature measurement.  Then polls until temperature is ready.
        // Then automatically starts a pressure measurement and polls for that
        // to complete.  When both measurement are complete and in the local
        // buffer then the application callback is called from the I2C
        // interrupt context.  Polling is done on I2C interrupts allowing
        // processor to continue doing other tasks as needed.
        //
        INA219ReadCurrentRaw(&g_sINA219Inst, INA219AppCallback, &g_sINA219Inst,INA219_REG_CURRENT);

        SysCtlDelay(2800);

        INA219ReadPowerRaw(&g_sINA219Inst, INA219AppCallback, &g_sINA219Inst,INA219_REG_POWER);

        INA219CalculateCurrent(&g_sINA219Inst, &fTemperature);

        while(g_vui8DataFlag == 0);

        //
        // Reset the data ready flag.
        //
        g_vui8DataFlag = 0;




        SysCtlDelay(12000000);
    }
}
