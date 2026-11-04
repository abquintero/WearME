/*
 * mpu6050.c
 *
 *  Created on: May 2, 2026
 *      Author: xbray
 */

#include "MPU6050.h"
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;

MPU6050_Data     mpu_data  = {0};
MPU6050_FR_State fr_state  = {0};

static MPU6050_BT_Callback bt_send_cb = NULL;

// EXPANSION TORACICA
float excursion_mm = 0.0f;

/*===============================================================================
						INICIALIZACIÓN DEL SENSOR
================================================================================*/

void MPU6050_Init(void)
{
	uint8_t temp_data;
	    HAL_StatusTypeDef ret;

	    /* Despertar el sensor */
	    temp_data = 0x00;
	    ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_PWR_MGMT_1, 1, &temp_data, 1, 100);
	    if (ret == HAL_OK) printf("Sensor despertado.\n");
	    else printf("Error al despertar el sensor.\n");

	    /* Verificar conexión */
	    ret = HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR, 3, 100);
	    if (ret == HAL_OK)
	        printf("MPU6050 conectado correctamente.\n");
	    else
	    {
	    	printf("ERROR: MPU6050 no encontrado.\n");
	        return;
	    }

	    /* Configuración giroscopio */
	    temp_data = FS_GYRO_500;
	    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_CONFIG_GYRO, 1, &temp_data, 1, 100);

	    /* Configuración acelerómetro */
	    temp_data = FS_ACC_4G;
	    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_CONFIG_ACC, 1, &temp_data, 1, 100);
}

void MPU6050_Read(void)
{
	uint8_t data[2];
	    int16_t raw;

	    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, REG_DATA_ACC_X, 1, data, 2, 100);
	    raw = ((int16_t)data[0] << 8) | data[1];
	    mpu_data.ax_raw = raw;
	    mpu_data.ax_ms2 = (raw / SENS_ACC_4G) * 9.81f;

	    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, REG_DATA_ACC_Y, 1, data, 2, 100);
	    raw = ((int16_t)data[0] << 8) | data[1];
	    mpu_data.ay_raw = raw;
	    mpu_data.ay_ms2 = (raw / SENS_ACC_4G) * 9.81f;

	    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, REG_DATA_ACC_Z, 1, data, 2, 100);
	    raw = ((int16_t)data[0] << 8) | data[1];
	    mpu_data.az_raw = raw;
	    mpu_data.az_ms2 = (raw / SENS_ACC_4G) * 9.81f;
}

/*===========================================================================
				      CALIBRACION DEL SENSOR
===========================================================================*/

void MPU6050_FR_Init(void)
{
    //	Limpiar historial de intervalos
    for (int i = 0; i < N_INTERVALOS; i++)
        fr_state.intervalos[i] = 0.0f;

    //	Lectura inicial
    MPU6050_Read();
    fr_state.az_suavizado = mpu_data.az_ms2;
    fr_state.az_baseline  = fr_state.az_suavizado;
    fr_state.t_muestra    = HAL_GetTick();

    printf("=========================================================\n");
    printf("    Calibrando... permanezca quieto por 2 segundos       \n");
    printf("=========================================================\n");
}

/*===========================================================================
					RECALCULAR FRECUENCIA RESPIRATORIA
===========================================================================*/

void recalcularFR(float excursion_mm)
{
    float suma = 0.0f;
    for (int i = 0; i < N_INTERVALOS; i++)
        suma += fr_state.intervalos[i];

    fr_state.frec_resp = 60000.0f / (suma / N_INTERVALOS);
    printf("FR = %.1f rpm\n", fr_state.frec_resp);

    //	Enviar por Bluetooth
    char bt_buf[20];
    snprintf(bt_buf, sizeof(bt_buf), "FR:%.1f\r\n", fr_state.frec_resp);
    BT_Send(bt_buf);
}

// CALLBACK DE BLUETOOTH
void MPU6050_FR_SetBTCallback(MPU6050_BT_Callback cb)
{
    bt_send_cb = cb;
}

/*===========================================================================
					CÁLCULOS DE FR Y ET
===========================================================================*/

void MPU6050_FR_Update(void)
{
    uint32_t ahora = HAL_GetTick();

    if (ahora - fr_state.t_muestra < INTERVALO_MS) return;
    fr_state.t_muestra = ahora;

    MPU6050_Read();
    float az_ms2 = mpu_data.az_ms2;

    /* ── Calibración inicial ───────────────────────────────────── */
    if (!fr_state.calibrado)
    {
        fr_state.muestras_cal++;
        fr_state.az_suavizado = ALPHA * az_ms2 + (1.0f - ALPHA) * fr_state.az_suavizado;
        fr_state.az_baseline  = fr_state.az_suavizado;

        if (fr_state.muestras_cal >= MUESTRAS_CAL)
        {
            fr_state.calibrado = 1;
            printf("Calibracion lista. Respira normalmente.\n");
            printf("-----------------------------\n");
        }
        return;
    }

    /* ── Acumular bloque de MUESTRAS_POR_BLOQUE ───────────────── */
    fr_state.acumulador += az_ms2;
    fr_state.contador_bloque++;
    if (fr_state.contador_bloque < MUESTRAS_POR_BLOQUE) return;

    float promedio          = fr_state.acumulador / MUESTRAS_POR_BLOQUE;
    fr_state.acumulador     = 0.0f;
    fr_state.contador_bloque = 0;
    ahora                   = HAL_GetTick();

    /* ── Filtros EMA + baseline + centrado ────────────────────── */
    fr_state.az_suavizado = ALPHA      * promedio              + (1.0f - ALPHA)      * fr_state.az_suavizado;
    fr_state.az_baseline  = ALPHA_BASE * fr_state.az_suavizado + (1.0f - ALPHA_BASE) * fr_state.az_baseline;
    fr_state.az_centrado  = fr_state.az_suavizado - fr_state.az_baseline;

    /* ── Detección de picos ────────────────────────────────────── */
    if (fr_state.buscando_pico)
    {
        if (fr_state.az_centrado > fr_state.valor_max)
            fr_state.valor_max = fr_state.az_centrado;

        if (fr_state.valor_max - fr_state.az_centrado > UMBRAL)
        {
            fr_state.buscando_pico = 0;
            fr_state.valor_min     = fr_state.az_centrado;
        }
    }
    else
    {
        if (fr_state.az_centrado < fr_state.valor_min)
            fr_state.valor_min = fr_state.az_centrado;

        if (fr_state.az_centrado - fr_state.valor_min > UMBRAL)
        {
            float amplitud = fr_state.valor_max - fr_state.valor_min;

            if (amplitud >= AMPLITUD_MIN && ahora - fr_state.t_ultimo_ciclo > TIEMPO_MIN_CICLO)
            {
                if (fr_state.primer_pico)
                {
                    fr_state.t_ultimo_pico = ahora;
                    fr_state.primer_pico   = 0;
                    if (bt_send_cb) bt_send_cb("RD\r\n");
                }
                else
                {
                    float intervalo_nuevo = (float)(ahora - fr_state.t_ultimo_pico);
                    fr_state.t_ultimo_pico = ahora;

                    fr_state.intervalos[fr_state.total_intervalos] = intervalo_nuevo;
                    fr_state.total_intervalos++;
                    if (bt_send_cb) bt_send_cb("RD\r\n");

                    excursion_mm = amplitud * 60.0f;
                    printf("Expansion toracica: %.1f mm\n", excursion_mm);

                    char bt_buf[30];
                    snprintf(bt_buf, sizeof(bt_buf), "ET:%.1f mm\n", excursion_mm);
                    BT_Send(bt_buf);

                    if (fr_state.total_intervalos >= N_INTERVALOS)
                    {
                    	recalcularFR(excursion_mm);
                    	fr_state.total_intervalos = 0;
                    }
                }
                fr_state.t_ultimo_ciclo = ahora;
            }

            fr_state.buscando_pico = 1;
            fr_state.valor_max     = fr_state.az_centrado;
            fr_state.valor_min     = 9999.0f;
        }
    }

    /* ── Timeout: reiniciar si no hay respiración detectada ───── */
    if (!fr_state.primer_pico && (ahora - fr_state.t_ultimo_pico) > TIMEOUT_RESP)
    {
        fr_state.frec_resp        = 0.0f;
        fr_state.total_intervalos = 0;
        fr_state.primer_pico      = 1;
        fr_state.valor_max        = -9999.0f;
        fr_state.valor_min        =  9999.0f;
        fr_state.buscando_pico    = 1;
        for (int i = 0; i < N_INTERVALOS; i++)
            fr_state.intervalos[i] = 0.0f;
        printf("! Timeout: reiniciando.\n");
    }
}
