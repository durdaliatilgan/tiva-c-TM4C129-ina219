#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "sensorlib/i2cm_drv.h"
#include "drivers/ina219.h"
#include "drivers/ina219_hw.h"

//*****************************************************************************
// The states of the INA219 state machine.
//*****************************************************************************
#define INA219_STATE_IDLE            0
#define INA219_STATE_INIT            1
#define INA219_STATE_READ            2
#define INA219_STATE_WRITE           3
#define INA219_STATE_RMW             4
#define INA219_STATE_INIT2           5
#define INA219_STATE_WAIT_CUR        6
#define INA219_STATE_READ_CUR        7
#define INA219_STATE_READ_POWER      8



//*****************************************************************************
// The callback function that is called when I2C transations to/from the INA219
// have completed.
//*****************************************************************************
static void (INA219Callback)(void *pvCallbackData, uint_fast8_t ui8Status)
{
    tINA219 *psInst;

    //
    // Convert the instance data into a pointer to a tINA219 structure.
    //
    psInst = pvCallbackData;

    //
    // If the I2C master driver encountered a failure, force the state machine
    // to the idle state (which will also result in a callback to propagate the
    // error).
    //
    if(ui8Status != I2CM_STATUS_SUCCESS)
    {
        psInst->ui8State = INA219_STATE_IDLE;
    }

    //
    // Determine the current state of the INA219 state machine.
    //
    switch(psInst->ui8State)
    {
        //
        // All states that trivially transition to IDLE, and all unknown
        // states.
        //
        case INA219_STATE_INIT:
        case INA219_STATE_INIT2:
        case INA219_STATE_READ:
        case INA219_STATE_WRITE:
        case INA219_STATE_RMW:


        default:
        {
            //
            // The state machine is now idle.
            //
            psInst->ui8State = INA219_STATE_IDLE;

            //
            // Done.
            //
            break;
        }


        //
        // Waiting for the temperature reading to be available.
        //
        case INA219_STATE_READ_CUR:
        {
            //
            // The temperature reading is ready, so read it now.
            //
            I2CMRead(psInst->psI2CInst, psInst->ui8Addr,
                     psInst->uCommand.pui8BufferCur, 1, psInst->pui8Data, 2,
                     INA219Callback, psInst);

            //
            // Move to the temperature reading state.
            //
            psInst->ui8State = INA219_STATE_IDLE;


            //
            // Done.
            //
            break;
        }
        case INA219_STATE_READ_POWER:
        {
            //
            // The temperature reading is ready, so read it now.
            //
            I2CMRead(psInst->psI2CInst, psInst->ui8Addr,
                     psInst->uCommand.pui8BufferPow, 1, psInst->pui8DataPower, 2,
                     INA219Callback, psInst);

            //
            // Move to the temperature reading state.
            //
            psInst->ui8State = INA219_STATE_IDLE;


            //
            // Done.
            //
            break;
        }



    }

    //
    // See if the state machine is now idle and there is a callback function.
    //
    if((psInst->ui8State == INA219_STATE_IDLE) && psInst->pfnCallback)
    {
        //
        // Call the application-supplied callback function.
        //
        psInst->pfnCallback(psInst->pvCallbackData, ui8Status);
    }
}

//*****************************************************************************
//! Initializes the INA219 driver.
//!
//! \param psInst is a pointer to the INA219 instance data.
//! \param psI2CInst is a pointer to the I2C driver instance data.
//! \param ui8I2CAddr is the I2C address of the INA219 device.
//! \param pfnCallback is the function to be called when the initialization has
//! completed (can be \b NULL if a callback is not required).
//! \param pvCallbackData is a pointer that is passed to the callback function.
//!
//! This function initializes the INA219 driver, preparing it for operation,
//! and initiates a reset of the INA219 device, clearing any previous
//! configuration data.
//!
//! \return Returns 1 if the INA219 driver was successfully initialized and 0
//! if it was not.
//*****************************************************************************
uint_fast8_t (INA219Calibrate)(tINA219 *psInst, tI2CMInstance *psI2CInst, uint_fast8_t ui8I2CAddr,
                        tSensorCallback *pfnCallback, void *pvCallbackData,uint8_t calibrateType)
{

    psInst->sCalibration.calibNumber = calibrateType;

    switch(psInst->sCalibration.calibNumber)
        {

            case INA219_CALIB_32V_2A:
            {
                psInst->sCalibration.ina219_calibrationValue=4096;
                psInst->sCalibration.ina219_currentDivider_mA = 100;
                psInst->sCalibration.ina219_powerMultiplier_mW= 2;
                psInst->sCalibration.configSettings = (INA219_CONFIG_BVOLTAGERANGE_32V |INA219_CONFIG_GAIN_8_320MV |
                                                        INA219_CONFIG_BADCRES_12BIT | INA219_CONFIG_SADCRES_12BIT_1S_532US |
                                                        INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS);
                break;

            }
            case INA219_CALIB_32V_1A:
            {
                psInst->sCalibration.ina219_calibrationValue=10240;
                psInst->sCalibration.ina219_currentDivider_mA = 25;
                psInst->sCalibration.ina219_powerMultiplier_mW= 1;
                psInst->sCalibration.configSettings = (INA219_CONFIG_BVOLTAGERANGE_32V | INA219_CONFIG_GAIN_8_320MV |
                                                        INA219_CONFIG_BADCRES_12BIT | INA219_CONFIG_SADCRES_12BIT_1S_532US |
                                                        INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS);
                break;
            }
            case INA219_CALIB_16V_400MA:
            {
                psInst->sCalibration.ina219_calibrationValue=8192;
                psInst->sCalibration.ina219_currentDivider_mA = 20;
                psInst->sCalibration.ina219_powerMultiplier_mW= 1;
                psInst->sCalibration.configSettings = (INA219_CONFIG_BVOLTAGERANGE_16V | INA219_CONFIG_GAIN_1_40MV |
                                                        INA219_CONFIG_BADCRES_12BIT | INA219_CONFIG_SADCRES_12BIT_1S_532US |
                                                        INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS);
                break;
            }

        }



    //
    // Initialize the INA219 instance structure
    //
    psInst->psI2CInst = psI2CInst;
    psInst->ui8Addr = ui8I2CAddr;
    psInst->ui8State = INA219_STATE_INIT;
    //
    // Save the callback information.
    //
    psInst->pfnCallback = pfnCallback;
    psInst->pvCallbackData = pvCallbackData;

    //
    // Write the configuration register to its default value.
    //pui8DataCalib[3];
    psInst->pui8DataCalib[0] = INA219_REG_CALIBRATION;
    psInst->pui8DataCalib[1] = (uint8_t)(psInst->sCalibration.ina219_calibrationValue >> 8);
    psInst->pui8DataCalib[2] = (uint8_t)(psInst->sCalibration.ina219_calibrationValue & 0xFF);
    //
    // Write the reset bit and issue a callback when finished.
    //
    if(I2CMWrite(psInst->psI2CInst, ui8I2CAddr, psInst->pui8DataCalib, 3,
                 INA219Callback, psInst) == 0  )
    {
        //
        // I2CMWrite failed so reset INA219 state and return zero to indicate
        // failure.
        //
        psInst->ui8State = INA219_STATE_IDLE;
        return(0);
    }


    //
    // Success
    //
    return(1);
}





uint_fast8_t (INA219Configuration)(tINA219 *psInst, tI2CMInstance *psI2CInst, uint_fast8_t ui8I2CAddr,
                        tSensorCallback *pfnCallback, void *pvCallbackData,uint8_t calibrateType)
{

    //
    // Initialize the INA219 instance structure
    //
    psInst->psI2CInst = psI2CInst;
    psInst->ui8Addr = ui8I2CAddr;
    psInst->ui8State = INA219_STATE_INIT2;


    //
    // Save the callback information.
    //
    psInst->pfnCallback = pfnCallback;
    psInst->pvCallbackData = pvCallbackData;

    //
    // Write the configuration register to its default value.
    //
    psInst->pui8DataConfig[0] = INA219_REG_CONFIG;
    psInst->pui8DataConfig[1] = (uint8_t)(psInst->sCalibration.configSettings >> 8);
    psInst->pui8DataConfig[2] = (uint8_t)(psInst->sCalibration.configSettings & 0xFF);
    //
    // Write the reset bit and issue a callback when finished.
    //
    if(I2CMWrite(psInst->psI2CInst, ui8I2CAddr, psInst->pui8DataConfig, 3,
                 INA219Callback, psInst) == 0  )
    {
        //
        // I2CMWrite failed so reset INA219 state and return zero to indicate
        // failure.
        //
        psInst->ui8State = INA219_STATE_IDLE;
        return(0);
    }
    //
    // Success
    //
    return(1);
}


//*****************************************************************************
//! Reads data from INA219 registers.
//!
//! \param psInst is a pointer to the INA219 instance data.
//! \param ui8Reg is the first register to read.
//! \param pui16Data is a pointer to the location to store the data that is
//! read.
//! \param ui16Count the number of register values to read.
//! \param pfnCallback is the function to be called when data read is complete
//! (can be \b NULL if a callback is not required).
//! \param pvCallbackData is a pointer that is passed to the callback function.
//!
//! This function reads a sequence of data values from consecutive registers in
//! the INA219.
//!
//! \note The INA219 does not auto-increment the register pointer, so reads of
//! more than one value returns garbage for the subsequent values.
//!
//! \return Returns 1 if the write was successfully started and 0 if it was
//! not.
//*****************************************************************************
uint_fast8_t (INA219Read)(tINA219 *psInst, uint_fast8_t ui8Reg, uint16_t *pui16Data,
           uint_fast16_t ui16Count, tSensorCallback *pfnCallback,
           void *pvCallbackData)
{
    //
    // Return a failure if the INA219 driver is not idle (in other words, there
    // is already an outstanding request to the INA219).
    //
    if(psInst->ui8State != INA219_STATE_IDLE)
    {
        return(0);
    }

    //
    // Save the callback information.
    //
    psInst->pfnCallback = pfnCallback;
    psInst->pvCallbackData = pvCallbackData;

    //
    // Move the state machine to the wait for read state.
    //
    psInst->ui8State = INA219_STATE_READ;

    //
    // Read the requested registers from the INA219.
    //
    if(ui8Reg == INA219_READ)
    {
        //
        // The configuration register is only one byte, so only a single byte
        // read is necessary and no endian swapping is required.
        //
        psInst->uCommand.pui8Buffer[0] = ui8Reg;
        if(I2CMRead(psInst->psI2CInst, psInst->ui8Addr,
                    psInst->uCommand.pui8Buffer, 1, (uint8_t *)pui16Data, 1,
                    INA219Callback, psInst) == 0)
        {
            //
            // The I2C write failed, so move to the idle state and return a
            // failure.
            //
            psInst->ui8State = INA219_STATE_IDLE;
            return(0);
        }
    }
    else
    {
        //
        // This is one of the temperature registers, which are 16-bit
        // big-endian registers.
        if(I2CMRead16BE(&(psInst->uCommand.sReadState), psInst->psI2CInst,
                        psInst->ui8Addr, ui8Reg, pui16Data, ui16Count,
                        INA219Callback, psInst) == 0)
        {
            //
            // The I2C write failed, so move to the idle state and return a
            // failure.
            //
            psInst->ui8State = INA219_STATE_IDLE;
            return(0);
        }
    }

    //
    // Success.
    //
    return(1);
}


uint_fast8_t (INA219ReadCurrentRaw)(tINA219 *psInst, tSensorCallback *pfnCallback,
               void *pvCallbackData, uint8_t readWhichValue)
{

    //
    // Return a failure if the INA219 driver is not idle (in other words, there
    // is already an outstanding request to the INA219).
    //
    if(psInst->ui8State != INA219_STATE_IDLE)
    {
        return(0);
    }

    //
    // Save the callback information.
    //
    psInst->pfnCallback = pfnCallback;
    psInst->pvCallbackData = pvCallbackData;

    //
    // Move the state machine to the wait for data read state.
    //
    psInst->ui8State = INA219_STATE_READ_CUR;

    psInst->uCommand.pui8BufferCur[0]= INA219_REG_CURRENT;
    //
    // Read the temperature data from the INA219.
    //
    if(I2CMWrite(psInst->psI2CInst, psInst->ui8Addr, psInst->pui8DataCalib, 3,
                 INA219Callback, psInst) == 0  )
    {
        //
        // I2CMWrite failed so reset INA219 state and return zero to indicate
        // failure.
        //
        psInst->ui8State = INA219_STATE_IDLE;
        return(0);
    }
    //
    // Success.
    //
    return(1);
}

uint_fast8_t (INA219ReadPowerRaw)(tINA219 *psInst, tSensorCallback *pfnCallback,
               void *pvCallbackData, uint8_t readWhichValue)
{

    //
    // Return a failure if the INA219 driver is not idle (in other words, there
    // is already an outstanding request to the INA219).
    //
    if(psInst->ui8State != INA219_STATE_IDLE)
    {
        return(0);
    }

    //
    // Save the callback information.
    //
    psInst->pfnCallback = pfnCallback;
    psInst->pvCallbackData = pvCallbackData;

    //
    // Move the state machine to the wait for data read state.
    //
    psInst->ui8State = INA219_STATE_READ_POWER;

    psInst->uCommand.pui8BufferPow[0]=INA219_REG_POWER;

    //
    // Read the temperature data from the INA219.
    //
    if(I2CMWrite(psInst->psI2CInst, psInst->ui8Addr, psInst->pui8DataCalib, 3,
                 INA219Callback, psInst) == 0  )
    {
        //
        // I2CMWrite failed so reset INA219 state and return zero to indicate
        // failure.
        //
        psInst->ui8State = INA219_STATE_IDLE;
        return(0);
    }
    //
    // Success.
    //
    return(1);
}
uint_fast8_t (INA219ReadBusVoltage)(tINA219 *psInst, tSensorCallback *pfnCallback,
               void *pvCallbackData)
{

    //
    // Return a failure if the INA219 driver is not idle (in other words, there
    // is already an outstanding request to the INA219).
    //
    if(psInst->ui8State != INA219_STATE_IDLE)
    {
        return(0);
    }

    //
    // Save the callback information.
    //
    psInst->pfnCallback = pfnCallback;
    psInst->pvCallbackData = pvCallbackData;

    //
    // Move the state machine to the wait for data read state.
    //
    psInst->ui8State = INA219_STATE_READ;

    //
    // Read the requested registers from the INA219180.
    //
    psInst->uCommand.pui8Buffer[0] = INA219_REG_BUSVOLTAGE;

    if(I2CMRead(psInst->psI2CInst, psInst->ui8Addr,
                psInst->uCommand.pui8Buffer, 1, psInst->pui8Data, 2,
                INA219Callback, psInst) == 0)
    {
        //
        // The I2C write failed, so move to the idle state and return a
        // failure.
        //
        psInst->ui8State = INA219_STATE_IDLE;
        return(0);
    }
    //
    // Success.
    //
    return(1);
}

uint_fast8_t (INA219ReadShontVoltage)(tINA219 *psInst, tSensorCallback *pfnCallback,
               void *pvCallbackData)
{

    //
    // Return a failure if the INA219 driver is not idle (in other words, there
    // is already an outstanding request to the INA219).
    //
    if(psInst->ui8State != INA219_STATE_IDLE)
    {
        return(0);
    }

    //
    // Save the callback information.
    //
    psInst->pfnCallback = pfnCallback;
    psInst->pvCallbackData = pvCallbackData;

    //
    // Move the state machine to the wait for data read state.
    //
    psInst->ui8State = INA219_STATE_READ;

    //
    // Read the requested registers from the INA219180.
    //
    psInst->uCommand.pui8Buffer[0] = INA219_REG_SHUNTVOLTAGE;

    if(I2CMRead(psInst->psI2CInst, psInst->ui8Addr,
                psInst->uCommand.pui8Buffer, 1, psInst->pui8Data, 2,
                INA219Callback, psInst) == 0)
    {
        //
        // The I2C write failed, so move to the idle state and return a
        // failure.
        //
        psInst->ui8State = INA219_STATE_IDLE;
        return(0);
    }
    //
    // Success.
    //
    return(1);
}

void (INA219CalculateCurrent)(tINA219 *psInst, float *pui16Current)
{
    //
    // Return the raw pressure value.
    //
    float asd= ((psInst->pui8Data[0] << 8) | (psInst->pui8Data[1] )) / (float) psInst->sCalibration.ina219_currentDivider_mA;
    *pui16Current = ((psInst->pui8Data[0] << 8) | (psInst->pui8Data[1] )) / (float) psInst->sCalibration.ina219_currentDivider_mA;

}
void (INA219CalculatePower)(tINA219 *psInst, float *pui16Power)
{
    //
    // Return the power value.
    //
    float asd = ((psInst->pui8DataPower[0] << 8) | (psInst->pui8DataPower[1] )) * psInst->sCalibration.ina219_powerMultiplier_mW;
    *pui16Power = ((psInst->pui8DataPower[0] << 8) | (psInst->pui8DataPower[1] )) * psInst->sCalibration.ina219_powerMultiplier_mW;
}
void (INA219CalculateShontVoltage)(tINA219 *psInst, float *pui16ShontVoltage)
{
    //
    // Return the raw pressure value.
    //
    *pui16ShontVoltage = ((psInst->pui8Data[0] << 8) | (psInst->pui8Data[1] )) * 0.01;
}
void (INA219CalculateBusVoltage)(tINA219 *psInst, float *pui16BusVoltage)
{
    //
    // Return the raw pressure value.
    //
    *pui16BusVoltage = ((psInst->pui8Data[0] << 8) | (psInst->pui8Data[1] )) * 0.001;;
}
