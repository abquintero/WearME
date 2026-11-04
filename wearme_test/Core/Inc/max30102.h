/*
 * max30102.h
 *
 *  Created on: May 1, 2026
 *      Author: xbray
 */

#ifndef INC_MAX30102_H_
#define INC_MAX30102_H_

#include "stm32g4xx_hal.h"

//	DEVICE ADDRESS
#define MAX_I2C_ADDRESS			0xAE

//	STATUS REGISTERS
#define INT_STATUS_1			0x00
#define INT_STATUS_2			0x01
#define INT_ENABLE_1			0x02
#define INT_ENABLE_2			0x03

//	FIFO REGISTERS
#define FIFO_WRITE_POINTER_REG	0x04
#define FIFO_OVERF_COUNTER_REG	0x05
#define FIFO_READ_POINTER_REG	0x06
#define FIFO_DATA_REG			0x07
#define	FIFO_ROLLOVER_EN		0x10
#define	FIFO_A_FULL				0x0F	// Interrumpir con FIFO con 17 muestras listas, 15 de margen
#define FIFO_SMP_AVG    		0x40  	// Promediado de 4 muestras

//	CONFIGURATION REGISTERS
#define FIFO_CONFIG_REG			0x08
#define MODE_CONFIG_REG			0x09
#define SPO2_CONFIG_REG			0x0A
#define LED_CONFIG_1_REG		0x0C
#define LED_CONFIG_2_REG		0x0D
#define MULTI_LED_CTRL1_REG		0x11
#define MULTI_LED_CTRL2_REG		0x12

//	SAMPLING PARAMETERS
#define MAX30102_SR_50HZ		0x00
#define MAX30102_SR_100HZ		0x04
#define MAX30102_SR_200HZ		0x08

#define MAX30102_ADC_RNG_2048	0x00
#define MAX30102_ADC_RNG_4096	0x20	//	Rango ADC
#define MAX30102_ADC_RNG_8192	0x40
#define MAX30102_ADC_RNG_16384	0x60

#define MAX30102_PW_15BIT		0x00
#define MAX30102_PW_16BIT		0x01
#define MAX30102_PW_17BIT		0x02
#define MAX30102_PW_18BIT		0x03   //	Máxima resolución

// VALORES USADOS EN MODE_CONFIG_REG
#define MODE_RESET				0x40
#define MODE_SPO2				0x03
#define MODE_HR					0x02

//	DIE TEMPERATURE REGISTERS
#define TEMP_INTEGER_REG		0x1F
#define TEMP_FRACTION_REG		0x20
#define TEMP_CONFIG_REG			0x21

//	PART ID REGISTERS
#define REVISION_ID_REG			0xFE
#define PART_ID_REG				0xFF

//	TAMAÑO DE MUESTRAS BUFFER
#define SPO2_WINDOW       		200		// # Muestras - Ventana larga  -> SpO2 (8s a 25Hz)
#define HR_WINDOW         		100		// # Muestras - Ventana corta  -> HR   (4s a 25Hz)
#define EFFECTIVE_SR      		25		// Muestras/segundo reales tras SMP_AVG=4
#define FIFO_DEPTH				32		// Num. Posiciones de memoria FIFO


//	ESTRUCTURA PRINCIPAL DE DRIVERS
typedef struct {
	//	ACCEDER A LOS CANALES I2C
	I2C_HandleTypeDef *hi2c;			//	Handle I2C (i2c1 o i2c2)

	// FIFO - ALMACENAR MUESTRAS
	uint32_t red_buf[SPO2_WINDOW];		//	Red led buffer
	uint32_t ir_buf[SPO2_WINDOW];		//	IR led buffer
	uint16_t buf_idx;					//	Indice (posición) del buffer circular
	uint8_t  spo2_buf_ready;           	//	1 cuando hay >= SPO2_WINDOW muestras válidas
    uint8_t  hr_buf_ready;       		//	1 cuando hay >= HR_WINDOW muestras válidas
    uint16_t sample_count;       		//	contador absoluto desde el inicio
	uint32_t last_red;					//	Ultima muestra cruda RED
	uint32_t last_ir;					//	Ultima muestra cruda IR
	uint8_t samples_this_burst;

	// SPO2 & HR signal
	int32_t  heart_rate;				//	Latidos por minuto (BPM)
	int32_t  spo2;						//	SpO2 en %
	uint8_t  hr_valid;					//	1 = HR confiable | 0 = no confiable
	uint8_t  spo2_valid;				//	1 = SpO2 confiable | 0 = no confiable

	//	HR BLOCK AVERAGING
	int32_t  hr_block_sum;    // suma acumulada del bloque actual
	uint8_t  hr_block_count;  // cuántos hr_calc lleva el bloque
	float    hr_display;      // valor filtrado entre bloques para mostrar

	// SPO2 BLOCK AVERAGING
	int32_t  spo2_block_sum;
	uint8_t  spo2_block_count;
	float    spo2_display;

} MAX30102_HANDLE;

//	PARÁMETROS PARA VERIFICAR I2C
typedef enum {
    MAX30102_OK    = 0,					// Responde correctamente I2C
    MAX30102_ERROR = 1,					// No responde en el bus I2C
	MAX30102_BUSSY = 2					// Responde, pero PART ID incorrecto
} MAX30102_STATUS;

//	FUNCIONES INTERNAS I2C
void MAX30102_I2C_Finder();
MAX30102_STATUS MAX30102_WriteReg(MAX30102_HANDLE *dev,uint8_t reg, uint8_t val);
MAX30102_STATUS MAX30102_ReadReg(MAX30102_HANDLE *dev, uint8_t reg, uint8_t *val);
MAX30102_STATUS MAX30102_ReadBytes(MAX30102_HANDLE *dev, uint8_t reg, uint8_t *buf, uint8_t len);

//	FUNCIONES DE INICIALIZACION
MAX30102_STATUS MAX30102_Reset(MAX30102_HANDLE *dev); //	NO SE ESTÁ USANDO
MAX30102_STATUS MAX30102_Init(MAX30102_HANDLE *dev, I2C_HandleTypeDef *hi2c);

//	FUNCIONES DE LECTURA FIFO
uint8_t MAX30102_ReadFIFO(MAX30102_HANDLE *dev);
void MAX30102_DebugPrint(MAX30102_HANDLE *dev);

//	FUNCIONES FISICAS DEL SENSOR
uint8_t MAX30102_FingerPresent(MAX30102_HANDLE *dev);	//	Detecta si hay dedo o no
uint32_t MAX30102_GetRed(MAX30102_HANDLE *dev);			//	Devuelve el último valor RED crudo (útil para debug)
uint32_t MAX30102_GetIR(MAX30102_HANDLE *dev);			//	Devuelve el último valor IR crudo (útil para debug)

//	FUNCION CALCULO DE SPO2 & HR
void MAX30102_Calc_HR(MAX30102_HANDLE *dev);
void MAX30102_Calc_SPO2(MAX30102_HANDLE *dev);
uint8_t MAX30102_SignalValid_HR(MAX30102_HANDLE *dev);
uint8_t MAX30102_SignalValid_SPO2(MAX30102_HANDLE *dev);

//	ENVIO BLUETOOTH
void BT_Send(const char* msg);

//	COMUNICACIÓN ITM
static inline void ITM_SendString(const char *str)
{
    while (*str)
    {
        ITM_SendChar((uint32_t)*str++);
    }
}

#endif /* INC_MAX30102_H_ */
