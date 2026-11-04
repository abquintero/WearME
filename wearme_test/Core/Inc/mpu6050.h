/*
 * mpu6050.h
 *
 *  Created on: May 2, 2026
 *      Author: xbray
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32g4xx_hal.h"

//	DIRECCION I2C DEL MPU
#define MPU6050_ADDR        0xD0

//	REGISTROS
#define REG_CONFIG_GYRO     27
#define REG_CONFIG_ACC      28
#define REG_PWR_MGMT_1      107
#define REG_DATA_ACC_X      59
#define REG_DATA_ACC_Y      61
#define REG_DATA_ACC_Z      63
#define REG_INT_PIN_CFG     55
#define REG_INT_ENABLE      56
#define REG_INT_STATUS      58

//	RANGOS DEL GIROSCOPIO
#define FS_GYRO_250         0x00
#define FS_GYRO_500         0x08
#define FS_GYRO_1000        0x10
#define FS_GYRO_2000        0x18

//	RANGOS DEL ACELEROMETRO
#define FS_ACC_2G           0x00
#define FS_ACC_4G           0x08
#define FS_ACC_8G           0x10
#define FS_ACC_16G          0x18
#define SENS_ACC_4G         8192.0f //	Sensibilidad para ±4g → 8192 LSB/g

//	PARÁMETROS ALGORITMO RESPIRATORIO
#define ALPHA				0.119f
#define ALPHA_BASE			0.005f
#define UMBRAL				0.03f
#define AMPLITUD_MIN		0.06f
#define TIEMPO_MIN_CICLO	1500UL
#define N_INTERVALOS		4
#define MUESTRAS_CAL		200
#define MUESTRAS_POR_BLOQUE	10
#define INTERVALO_MS		10UL
#define TIMEOUT_RESP		25000UL

//	DATOS CRUDOS DEL ACELEROMETRO
typedef struct {
	int16_t ax_raw, ay_raw, az_raw;
	float   ax_ms2, ay_ms2, az_ms2;
} MPU6050_Data;

//	ESTADO DEL ALGORITMO
typedef struct {
	float az_suavizado;
	float az_baseline;
	float az_centrado;
	//	Detección de picos
	float valor_max;
	float valor_min;
	uint8_t buscando_pico;
	uint32_t t_ultimo_ciclo;
	//	Historial de intervalos
	float intervalos[N_INTERVALOS];
	int total_intervalos;
	uint32_t t_ultimo_pico;
	uint8_t primer_pico;
	float frec_resp;
	//	Calibración
	uint8_t calibrado;
	int muestras_cal;
	float acumulador;
	int contador_bloque;
	uint32_t t_muestra;
} MPU6050_FR_State;

//	VARIABLES GLOBALES
extern MPU6050_Data 	mpu_data;
extern MPU6050_FR_State	fr_state;

//	ENVIO BLUETOOTH
typedef void (*MPU6050_BT_Callback)(const char* msg);

//	FUNCIONES
void MPU6050_Init(void);
void MPU6050_Read(void);
void RecalcularFR(void);
void MPU6050_FR_Init(void);
void MPU6050_FR_Update(void);
void BT_Send(const char* msg);
void MPU6050_FR_SetBTCallback(MPU6050_BT_Callback cb);

#endif /* INC_MPU6050_H_ */

