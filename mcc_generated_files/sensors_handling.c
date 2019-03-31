/*
    \file   sensors_handling.c

    \brief  Sensors handling handler source file.

    (c) 2018 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip software and any
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party
    license terms applicable to your use of third party software (including open source software) that
    may accompany Microchip software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS
    FOR A PARTICULAR PURPOSE.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS
    SOFTWARE.
*/

#include <stdint.h>
#include "sensors_handling.h"
#include "include/adc0.h"
#include "drivers/i2c_simple_master.h"
#include "string.h"
#define MCP9809_ADDR 0x18
#define MCP9808_REG_TA 0x05
#define LIGHT_SENSOR_ADC_CHANNEL 5

#define SPECTRO_TRIAD_SLV_ADDR 0x49


#define AS7265X_ADDR 0x49 //7-bit unshifted default I2C Address

#define AS7265X_STATUS_REG    0x00
#define AS7265X_WRITE_REG   0X01
#define AS7265X_READ_REG    0x02

#define AS7265X_TX_VALID    0x02
#define AS7265X_RX_VALID    0x01

//Register addresses
#define AS7265X_HW_VERSION_HIGH   0x00
#define AS7265X_HW_VERSION_LOW    0x01

#define AS7265X_FW_VERSION_HIGH  0x02
#define AS7265X_FW_VERSION_LOW  0x03

#define AS7265X_CONFIG      0x04
#define AS7265X_INTERGRATION_TIME 0x05
#define AS7265X_DEVICE_TEMP     0x06
#define AS7265X_LED_CONFIG    0x07

//Raw channel registers
#define AS7265X_R_G_A     0x08
#define AS7265X_S_H_B     0x0A
#define AS7265X_T_I_C     0x0C
#define AS7265X_U_J_D     0x0E
#define AS7265X_V_K_E     0x10
#define AS7265X_W_L_F     0x12

//Calibrated channel registers
#define AS7265X_R_G_A_CAL   0x14
#define AS7265X_S_H_B_CAL   0x18
#define AS7265X_T_I_C_CAL   0x1C
#define AS7265X_U_J_D_CAL   0x20
#define AS7265X_V_K_E_CAL   0x24
#define AS7265X_W_L_F_CAL   0x28

#define AS7265X_DEV_SELECT_CONTROL  0x4F

#define AS7265X_COEF_DATA_0   0x50
#define AS7265X_COEF_DATA_1   0x51
#define AS7265X_COEF_DATA_2   0x52
#define AS7265X_COEF_DATA_3   0x53
#define AS7265X_COEF_DATA_READ    0x54
#define AS7265X_COEF_DATA_WRITE   0x55

//Settings

#define AS7265X_POLLING_DELAY 5 //Amount of ms to wait between checking for virtual register changes

#define AS72651_NIR     0x00
#define AS72652_VISIBLE     0x01
#define AS72653_UV      0x02

#define AS7265x_LED_WHITE	0x00 //White LED is connected to x51
#define AS7265x_LED_IR	0x01 //IR LED is connected to x52
#define AS7265x_LED_UV	0x02 //UV LED is connected to x53

#define AS7265X_LED_CURRENT_LIMIT_12_5MA  0b00
#define AS7265X_LED_CURRENT_LIMIT_25MA    0b01
#define AS7265X_LED_CURRENT_LIMIT_50MA    0b10
#define AS7265X_LED_CURRENT_LIMIT_100MA   0b11

#define AS7265X_INDICATOR_CURRENT_LIMIT_1MA   0b00
#define AS7265X_INDICATOR_CURRENT_LIMIT_2MA   0b01
#define AS7265X_INDICATOR_CURRENT_LIMIT_4MA   0b10
#define AS7265X_INDICATOR_CURRENT_LIMIT_8MA   0b11

#define AS7265X_GAIN_1X   0b00
#define AS7265X_GAIN_37X   0b01
#define AS7265X_GAIN_16X   0b10
#define AS7265X_GAIN_64X   0b11

#define AS7265X_MEASUREMENT_MODE_4CHAN   0b00
#define AS7265X_MEASUREMENT_MODE_4CHAN_2   0b01
#define AS7265X_MEASUREMENT_MODE_6CHAN_CONTINUOUS   0b10
#define AS7265X_MEASUREMENT_MODE_6CHAN_ONE_SHOT   0b11

uint8_t virtualReadRegister(uint8_t virtualAddr)
{
	uint8_t status;

	//Do a prelim check of the read register
	status = i2c_read1ByteRegister(SPECTRO_TRIAD_SLV_ADDR,AS7265X_STATUS_REG);
	if ((status & AS7265X_RX_VALID) != 0) //There is data to be read
	{
		uint8_t incoming = i2c_read1ByteRegister(SPECTRO_TRIAD_SLV_ADDR,AS7265X_READ_REG); //Read the byte but do nothing with it
	}

	//Wait for WRITE flag to clear
	while (1)
	{
		status = i2c_read1ByteRegister(SPECTRO_TRIAD_SLV_ADDR,AS7265X_STATUS_REG);
		if ((status & AS7265X_TX_VALID) == 0) break; // If TX bit is clear, it is ok to write
		//delay(AS7265X_POLLING_DELAY);
	}

	// Send the virtual register address (bit 7 should be 0 to indicate we are reading a register).
	i2c_write1ByteRegister(SPECTRO_TRIAD_SLV_ADDR,AS7265X_WRITE_REG, virtualAddr);

	//Wait for READ flag to be set
	while (1)
	{
		status = i2c_read1ByteRegister(SPECTRO_TRIAD_SLV_ADDR,AS7265X_STATUS_REG);
		if ((status & AS7265X_RX_VALID) != 0) break; // Read data is ready.
		//delay(AS7265X_POLLING_DELAY);
	}

	uint8_t incoming = i2c_read1ByteRegister(SPECTRO_TRIAD_SLV_ADDR,AS7265X_READ_REG);
	return (incoming);
}

//Write to a virtual register in the AS726x
void virtualWriteRegister(uint8_t virtualAddr, uint8_t dataToWrite)
{
	uint8_t status;

	//Wait for WRITE register to be empty
	while (1)
	{
		status = i2c_read1ByteRegister(SPECTRO_TRIAD_SLV_ADDR,AS7265X_STATUS_REG);
		if ((status & AS7265X_TX_VALID) == 0) break; // No inbound TX pending at slave. Okay to write now.
		//delay(AS7265X_POLLING_DELAY);
	}

	// Send the virtual register address (setting bit 7 to indicate we are writing to a register).
	i2c_write1ByteRegister(SPECTRO_TRIAD_SLV_ADDR,AS7265X_WRITE_REG, (virtualAddr | 1<<7));

	//Wait for WRITE register to be empty
	while (1)
	{
		status = i2c_read1ByteRegister(SPECTRO_TRIAD_SLV_ADDR, AS7265X_STATUS_REG);
		if ((status & AS7265X_TX_VALID) == 0) break; // No inbound TX pending at slave. Okay to write now.
		//delay(AS7265X_POLLING_DELAY);
	}

	// Send the data to complete the operation.
	i2c_write1ByteRegister(SPECTRO_TRIAD_SLV_ADDR,AS7265X_WRITE_REG, dataToWrite);
}



void selectDevice(uint8_t device) {
	//Set the bits 0:1. Just overwrite whatever is there because masking in the correct value doesn't work.
	virtualWriteRegister(AS7265X_DEV_SELECT_CONTROL, device);

	//This fails
	//uint8_t value = virtualReadRegister(AS7265X_DEV_SELECT_CONTROL);
	//value &= 0b11111100; //Clear lower two bits
	//if(device < 3) value |= device; //Set the bits
	//virtualWriteRegister(AS7265X_DEV_SELECT_CONTROL, value);
}

//Given 4 bytes returns the floating point value
float convertBytesToFloat(uint32_t myLong)
{
	float myFloat;
	memcpy(&myFloat, &myLong, 4); //Copy bytes into a float
	return (myFloat);
}

float getCalibratedValue(uint8_t calAddress, uint8_t device)
{
	selectDevice(device);

	uint8_t b0, b1, b2, b3;
	b0 = virtualReadRegister(calAddress + 0);
	b1 = virtualReadRegister(calAddress + 1);
	b2 = virtualReadRegister(calAddress + 2);
	b3 = virtualReadRegister(calAddress + 3);

	//Channel calibrated values are stored big-endian
	uint32_t calBytes = 0;
	calBytes |= ((uint32_t)b0 << (8 * 3));
	calBytes |= ((uint32_t)b1 << (8 * 2));
	calBytes |= ((uint32_t)b2 << (8 * 1));
	calBytes |= ((uint32_t)b3 << (8 * 0));

	return (convertBytesToFloat(calBytes));
}

//Enable the LED or bulb on a given device
void enableBulb(uint8_t device)
{
	selectDevice(device);

	//Read, mask/set, write
	uint8_t value = virtualReadRegister(AS7265X_LED_CONFIG);
	value |= (1 << 3); //Set the bit
	virtualWriteRegister(AS7265X_LED_CONFIG, value);
}

//Disable the LED or bulb on a given device
void disableBulb(uint8_t device)
{
	selectDevice(device);

	//Read, mask/set, write
	uint8_t value = virtualReadRegister(AS7265X_LED_CONFIG);
	value &= ~(1 << 3); //Clear the bit
	virtualWriteRegister(AS7265X_LED_CONFIG, value);
}


//Mode 0: 4 channels out of 6 (see datasheet)
//Mode 1: Different 4 channels out of 6 (see datasheet)
//Mode 2: All 6 channels continuously
//Mode 3: One-shot reading of all channels
void setMeasurementMode(uint8_t mode)
{
	if (mode > 0b11) mode = 0b11; //Error check

	//Read, mask/set, write
	uint8_t value = virtualReadRegister(AS7265X_CONFIG); //Read
	value &= 0b11110011; //Clear BANK bits
	value |= (mode << 2); //Set BANK bits with user's choice
	virtualWriteRegister(AS7265X_CONFIG, value); //Write
}

//Checks to see if DRDY flag is set in the control setup register
uint8_t dataAvailable()
{
	uint8_t value = virtualReadRegister(AS7265X_CONFIG);
	return (value & (1 << 1)); //Bit 1 is DATA_RDY
}

void delay(uint8_t delay)
{
	for(uint8_t i=0; i<200; i++)
	{
		//for(uint8_t j=0; j < 100; j++);
	}
}

//Tells IC to take all channel measurements and polls for data ready flag
void takeMeasurements()
{
	setMeasurementMode(AS7265X_MEASUREMENT_MODE_6CHAN_ONE_SHOT); //Set mode to all 6-channels, one-shot

	//Wait for data to be ready
	while (dataAvailable() == 0) ;//delay(AS7265X_POLLING_DELAY);

	//Readings can now be accessed via getCalibratedA(), getJ(), etc
}

void takeMeasurementsWithBulb()
{
	enableBulb(AS7265x_LED_WHITE);
	enableBulb(AS7265x_LED_IR);
	enableBulb(AS7265x_LED_UV);

	takeMeasurements();

	disableBulb(AS7265x_LED_WHITE); //Turn off bulb to avoid heating sensor
	disableBulb(AS7265x_LED_IR);
	disableBulb(AS7265x_LED_UV);
}

void get_hw_info()
{
	uint8_t read_data;
	read_data = virtualReadRegister(AS7265X_HW_VERSION_HIGH);
	printf("Device type - %X\n",read_data);
	
	read_data = virtualReadRegister(AS7265X_HW_VERSION_LOW);
	printf("Hardware version - %X\n",read_data);
	
	virtualWriteRegister(AS7265X_FW_VERSION_HIGH,0x01);
	virtualWriteRegister(AS7265X_FW_VERSION_LOW,0x01);
	read_data = virtualReadRegister(AS7265X_FW_VERSION_LOW);
	printf("Major FW version - %X\n",read_data);
	
	virtualWriteRegister(AS7265X_FW_VERSION_HIGH,0x02);
	virtualWriteRegister(AS7265X_FW_VERSION_LOW,0x02);
	read_data = virtualReadRegister(AS7265X_HW_VERSION_LOW);
	printf("Patch FW version - %X\n",read_data);

	virtualWriteRegister(AS7265X_FW_VERSION_HIGH,0x03);
	virtualWriteRegister(AS7265X_FW_VERSION_LOW,0x03);
	read_data = virtualReadRegister(AS7265X_HW_VERSION_LOW);
	printf("Build FW version - %X\n",read_data);
}

void printfloat( float printval)
{
	printf(" %d.%d\n", (int)printval, (int)(1000*(printval -(int)printval)));
}

uint16_t getChannel(uint8_t channelRegister, uint8_t device)
{
	selectDevice(device);
	uint16_t colorData = virtualReadRegister(channelRegister) << 8; //High uint8_t
	colorData |= virtualReadRegister(channelRegister + 1); //Low uint8_t
	return (colorData);
}


void readRawData()
{
	uint16_t calib = 0;
	
	takeMeasurementsWithBulb();	
	
	calib = getChannel(AS7265X_R_G_A_CAL, AS72653_UV);
	printf("%d\n",calib);
			 
	calib = getChannel(AS7265X_S_H_B_CAL, AS72653_UV);
	printf("%d\n",calib);
			 
	calib = getChannel(AS7265X_T_I_C_CAL, AS72653_UV);
	printf("%d\n",calib);
			 
	calib = getChannel(AS7265X_U_J_D_CAL, AS72653_UV);
	printf("%d\n",calib);
	
	calib = getChannel(AS7265X_V_K_E_CAL, AS72653_UV);
	printf("%d\n",calib);
			 
	calib = getChannel(AS7265X_W_L_F_CAL, AS72653_UV);
	printf("%d\n",calib);
	
	calib = getChannel(AS7265X_R_G_A_CAL, AS72652_VISIBLE);
	printf("%d\n",calib);
			 
	calib = getChannel(AS7265X_S_H_B_CAL, AS72652_VISIBLE);
	printf("%d\n",calib);
			 
	calib = getChannel(AS7265X_T_I_C_CAL, AS72652_VISIBLE);
	printf("%d\n",calib);
			 
	calib = getChannel(AS7265X_U_J_D_CAL, AS72652_VISIBLE);
	printf("%d\n",calib);
			 
	calib = getChannel(AS7265X_V_K_E_CAL, AS72652_VISIBLE);
	printf("%d\n",calib);
	
	calib = getChannel(AS7265X_W_L_F_CAL, AS72652_VISIBLE);
	printf("%d\n",calib);
	
	calib = getChannel(AS7265X_R_G_A_CAL, AS72651_NIR);
	printf("%d\n",calib);
	
	calib = getChannel(AS7265X_S_H_B_CAL, AS72651_NIR);
	printf("%d\n",calib);
			 
	calib = getChannel(AS7265X_T_I_C_CAL, AS72651_NIR);
	printf("%d\n",calib);
			 
	calib = getChannel(AS7265X_U_J_D_CAL, AS72651_NIR);
	printf("%d\n",calib);
	
	calib = getChannel(AS7265X_V_K_E_CAL, AS72651_NIR);
	printf("%d\n",calib);
			 
	calib = getChannel(AS7265X_W_L_F_CAL, AS72651_NIR);
	printf("%d\n",calib);
	printf("\n\n\n\n\n\n\n\n\n\n\n");
}

void read_calib_values(sensor_data *dataset)
{
	//takeMeasurements();
	takeMeasurementsWithBulb();	
	//float calib = 0.0f;
	
	dataset->uv[0] = getCalibratedValue(AS7265X_R_G_A_CAL, AS72653_UV);
	//printfloat(calib);
	
	dataset->uv[1] = getCalibratedValue(AS7265X_S_H_B_CAL, AS72653_UV);
//	printfloat(calib);
	
	dataset->uv[2] = getCalibratedValue(AS7265X_T_I_C_CAL, AS72653_UV);
//	printfloat(calib);
	
	dataset->uv[3] = getCalibratedValue(AS7265X_U_J_D_CAL, AS72653_UV);
//	printfloat(calib);
	
	dataset->uv[4] = getCalibratedValue(AS7265X_V_K_E_CAL, AS72653_UV);
//	printfloat(calib);
	
	dataset->uv[5] = getCalibratedValue(AS7265X_W_L_F_CAL, AS72653_UV);
//	printfloat(calib);

	dataset->vs[0] = getCalibratedValue(AS7265X_R_G_A_CAL, AS72652_VISIBLE);
//	printfloat(calib);
	
	dataset->vs[1] = getCalibratedValue(AS7265X_S_H_B_CAL, AS72652_VISIBLE);
//	printfloat(calib);
	
	dataset->vs[2] = getCalibratedValue(AS7265X_T_I_C_CAL, AS72652_VISIBLE);
//	printfloat(calib);
	
	dataset->vs[3] = getCalibratedValue(AS7265X_U_J_D_CAL, AS72652_VISIBLE);
//	printfloat(calib);
	
	dataset->vs[4] = getCalibratedValue(AS7265X_V_K_E_CAL, AS72652_VISIBLE);
//	printfloat(calib);
	
	dataset->vs[5] = getCalibratedValue(AS7265X_W_L_F_CAL, AS72652_VISIBLE);
//	printfloat(calib);

	dataset->ir[0] = getCalibratedValue(AS7265X_R_G_A_CAL, AS72651_NIR);
//	printfloat(calib);
	
	dataset->ir[1] = getCalibratedValue(AS7265X_S_H_B_CAL, AS72651_NIR);
//	printfloat(calib);
	
	dataset->ir[2] = getCalibratedValue(AS7265X_T_I_C_CAL, AS72651_NIR);
//	printfloat(calib);
	
	dataset->ir[3] = getCalibratedValue(AS7265X_U_J_D_CAL, AS72651_NIR);
//	printfloat(calib);

	dataset->ir[4] = getCalibratedValue(AS7265X_V_K_E_CAL, AS72651_NIR);
//	printfloat(calib);
	
	dataset->ir[5] = getCalibratedValue(AS7265X_W_L_F_CAL, AS72651_NIR);
//	printfloat(calib);
	
//	printf("\n\n\n\n\n\n\n\n\n\n\n");
//	return (uint8_t)calib * 100;
}

#define MCP9809_ADDR				0x18 
#define MCP9808_REG_TA				0x05
#define LIGHT_SENSOR_ADC_CHANNEL	5

uint16_t SENSORS_getLightValue(void)
{
    return ADC_0_get_conversion(LIGHT_SENSOR_ADC_CHANNEL);
}

int16_t SENSORS_getTempValue (void)
{
    int32_t temperature;
    
    temperature = i2c_read2ByteRegister(MCP9809_ADDR, MCP9808_REG_TA);
    
    temperature = temperature << 19;
    temperature = temperature >> 19;
    
    temperature *= 100;
    temperature /= 16;
    
    return temperature;
}
