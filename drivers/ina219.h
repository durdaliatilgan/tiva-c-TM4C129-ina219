#ifndef DRIVERS_INA219_H_
#define DRIVERS_INA219_H_



#define INA219_CALIB_32V_2A     0
#define INA219_CALIB_32V_1A     1
#define INA219_CALIB_16V_400MA  2


typedef struct
{
	// The pointer to the I2C master interface instance used to communicate
	// with the INA219.
	tI2CMInstance *psI2CInst;


	// The I2C address of the INA219.
	uint8_t ui8Addr;


	// The state of the state machine used while accessing the INA219.
	uint8_t ui8State;

	// The data buffer used for sending/receiving data to/from the INA219.
	uint8_t pui8Data[2];

	// The data buffer used for sending/receiving data to/from the INA219.
	uint8_t pui8DataPower[2];

	// The data buffer used for calibration to the INA219.
	uint8_t pui8DataCalib[3];

	// The data buffer used for configuration to the INA219.
	uint8_t pui8DataConfig[3];

	// The function that is called when the current request has completed
	//
	tSensorCallback *pfnCallback;

	//
	// The pointer provided to the callback function.
	void *pvCallbackData;

	// A union of structures that are used for read, write and
	// read-modify-write operations.  Since only one operation can be active at
	// a time, it is safe to re-use the memory in this manner.
	struct
	{

		//
		// A buffer used to store the write portion of a register read.
		uint8_t pui8Buffer[3];
		//
		// A buffer used to store the write portion of a register read.
		uint8_t pui8BufferPow[3];

		//
		// A buffer used to store the write portion of a register read.
		uint8_t pui8BufferCur[3];

		//
		// The read state used to read register values.
		tI2CMRead16BE sReadState;

		//
		// The write state used to write register values.
		tI2CMWrite16BE sWriteState;

		//
		// The read-modify-write state used to modify 8-bit register values.
		tI2CMReadModifyWrite8 sReadModifyWriteState8;

		//
		// The read-modify-write state used to modify 16-bit register values.
		tI2CMReadModifyWrite16 sReadModifyWriteState16;
	}uCommand;
	struct
	{
		//
		//
		//
		uint16_t ina219_calibrationValue;
		//
		//
		//
		uint16_t ina219_currentDivider_mA;
		//
		//
		//
		uint16_t ina219_powerMultiplier_mW;
		//
		//
		//
		uint16_t calibNumber;
		//
		//
		//
		uint16_t configSettings;

	}sCalibration;
}
tINA219;




//*****************************************************************************
//
// Function prototypes.
//
//*****************************************************************************
extern uint_fast8_t INA219Calibrate(tINA219 *psInst, tI2CMInstance *psI2CInst, uint_fast8_t ui8I2CAddr,
		tSensorCallback *pfnCallback, void *pvCallbackData,uint8_t calibrateType);
extern uint_fast8_t INA219Configuration(tINA219 *psInst, tI2CMInstance *psI2CInst, uint_fast8_t ui8I2CAddr,
		tSensorCallback *pfnCallback, void *pvCallbackData,uint8_t calibrateType);
extern uint_fast8_t INA219Read(tINA219 *psInst, uint_fast8_t ui8Reg, uint16_t *pui16Data,
		uint_fast16_t ui16Count, tSensorCallback *pfnCallback, void *pvCallbackData);
extern uint_fast8_t INA219ReadCurrentRaw(tINA219 *psInst, tSensorCallback *pfnCallback,
		void *pvCallbackData, uint8_t readWhichValue);
extern uint_fast8_t INA219ReadBusVoltage(tINA219 *psInst, tSensorCallback *pfnCallback, void *pvCallbackData);
extern uint_fast8_t INA219ReadShontVoltage(tINA219 *psInst, tSensorCallback *pfnCallback, void *pvCallbackData);
extern uint_fast8_t (INA219ReadPowerRaw)(tINA219 *psInst, tSensorCallback *pfnCallback,
		void *pvCallbackData, uint8_t readWhichValue);

extern void INA219CalculateCurrent(tINA219 *psInst, float *pui16Current);
extern void INA219CalculatePower(tINA219 *psInst, float *pui16Power);
extern void INA219CalculateBusVoltage(tINA219 *psInst, float *pui16BusVoltage);
extern void INA219CalculateShontVoltage(tINA219 *psInst, float *pui16ShontVoltage);


#endif /* DRIVERS_INA219_H_ */
