/*
 * max30102.c
 *
 *  Created on: May 1, 2026
 *      Author: xbray
 */

#include <main.h>
#include <max30102.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

extern I2C_HandleTypeDef	hi2c3;
#define I2C_TIMEOUT			100

/*===============================================================================
				COMPROBAR REGISTRO DE COMUNICACIÓN I2C CON EL SENSOR
================================================================================*/

void MAX30102_I2C_Finder()
{
	char Finder[64];
	uint8_t encontrados = 0;
	for (uint16_t addr = 0x00; addr <= 0xFE; addr += 2)
	{
		if (HAL_I2C_IsDeviceReady(&hi2c3, addr, 1, 10) == HAL_OK)
		{
			sprintf(Finder, "Dispositivo MAX30102 encontrado en: 0x%02X\n", addr);
			ITM_SendString(Finder);
			encontrados++;
		}
	}

	if (encontrados == 0)
	{
		ITM_SendString("No se encontro MAX30102 en el bus I2C3\n");
	}
}

/*===============================================================================
 	 	 	 	 	 FUNCIONES DE BAJO NIVEL PARA VERIFICAR I2C
================================================================================*/

MAX30102_STATUS MAX30102_WriteReg(MAX30102_HANDLE *dev, uint8_t reg, uint8_t val)
{
	uint8_t buf[2] = { reg, val };
	if (HAL_I2C_Master_Transmit(dev->hi2c, MAX_I2C_ADDRESS, buf, 2, I2C_TIMEOUT) != HAL_OK)
		return MAX30102_ERROR;
	return MAX30102_OK;
}

MAX30102_STATUS MAX30102_ReadReg(MAX30102_HANDLE *dev, uint8_t reg, uint8_t *val)
{
	if (HAL_I2C_Master_Transmit(dev->hi2c, MAX_I2C_ADDRESS, &reg, 1, I2C_TIMEOUT) != HAL_OK)
		return MAX30102_ERROR;
	if (HAL_I2C_Master_Receive(dev->hi2c, MAX_I2C_ADDRESS, val, 1, I2C_TIMEOUT) != HAL_OK)
		return MAX30102_ERROR;
	return MAX30102_OK;
}

MAX30102_STATUS MAX30102_ReadBytes(MAX30102_HANDLE *dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
	if (HAL_I2C_Master_Transmit(dev->hi2c, MAX_I2C_ADDRESS, &reg, 1, I2C_TIMEOUT) != HAL_OK)
		return MAX30102_ERROR;
	if (HAL_I2C_Master_Receive(dev->hi2c, MAX_I2C_ADDRESS, buf, len, I2C_TIMEOUT) != HAL_OK)
		return MAX30102_ERROR;
	return MAX30102_OK;
}

/*===============================================================================
						INICIALIZACIÓN DEL SENSOR MAX30102
================================================================================*/

MAX30102_STATUS MAX30102_Reset(MAX30102_HANDLE *dev)
{
	MAX30102_STATUS s = MAX30102_WriteReg(dev, MODE_CONFIG_REG, MODE_RESET);
	HAL_Delay(100);
	return s;
}

MAX30102_STATUS MAX30102_Init(MAX30102_HANDLE *dev, I2C_HandleTypeDef *hi2c)

{
	uint8_t part_id = 0;

	//	LIMPIAR ESTRUCTURA (MAX30102_HANDLE)
	memset(dev, 0, sizeof(MAX30102_HANDLE));
	dev->hi2c = hi2c;

	//	VERIFICAR RESPUESTA DEL SENSOR
	if(MAX30102_ReadReg(dev, PART_ID_REG, &part_id)!= MAX30102_OK)
		return MAX30102_ERROR;
	if(part_id != 0x15)
		return MAX30102_ERROR;

	//	RESET DEL SENSOR
	if(MAX30102_Reset(dev) != MAX30102_OK)
		return MAX30102_ERROR;

	//	HABILITAR INTERRUPCIONES CUANDO DETECTA DATOS, SOLO A_FULL (FIFO Almost Full Flag)
	MAX30102_WriteReg(dev, INT_ENABLE_1, 0x80);
	MAX30102_WriteReg(dev, INT_ENABLE_2, 0x00);

	//	CONFIGURACION MUESTREO DEL FIFO
	MAX30102_WriteReg(dev, FIFO_CONFIG_REG, FIFO_SMP_AVG | FIFO_ROLLOVER_EN | FIFO_A_FULL);

	//	ACTIVACION MODO SPO2 (LED RED + IR => active)
	MAX30102_WriteReg(dev, MODE_CONFIG_REG, MODE_SPO2);

	// CONFIGURACION DEL MODO SPO2 (Rango ADC, Frecuencia de muestreo y resolución)
	MAX30102_WriteReg(dev, SPO2_CONFIG_REG, MAX30102_ADC_RNG_4096 | MAX30102_SR_100HZ | MAX30102_PW_18BIT);

	//	CONFIGURACION CORRIENTE DE LEDS - Rango: 0x00 (0 mA) a 0xFF (51 mA)
	MAX30102_WriteReg(dev, LED_CONFIG_1_REG, 0x1F); //	LED RED		~6.2mA
	MAX30102_WriteReg(dev, LED_CONFIG_2_REG, 0x1F);	//	IR			~6.2mA

    // Leer INT_STATUS_1: limpia cualquier interrupción pendiente del sensor
	uint8_t clean;
	MAX30102_ReadReg(dev, INT_STATUS_1, &clean);

	uint8_t wr_ptr;
	MAX30102_ReadReg(dev, FIFO_WRITE_POINTER_REG, &wr_ptr);
	MAX30102_WriteReg(dev, FIFO_READ_POINTER_REG,  wr_ptr);
	MAX30102_WriteReg(dev, FIFO_OVERF_COUNTER_REG, 0x00);

	//	PROMEDIADO DE 4 DATOS POR BLOQUE
	dev->hr_block_sum   = 0.0f;
	dev->hr_block_count = 0.0f;
	dev->hr_display     = 0.0f;
	dev->sample_count	= 0;
	return MAX30102_OK;
}

/*===============================================================================
							LECTURA DEL FIFO
=================================================================================
	Cada muestra del FIFO ocupa 6 bytes:
	[0][1][2] = 18 bits del canal Rojo  (MSB primero, bits [17:0])
	[3][4][5] = 18 bits del canal IR    (MSB primero, bits [17:0])
================================================================================*/

uint8_t MAX30102_ReadFIFO(MAX30102_HANDLE *dev)
{
    uint8_t wr_ptr, rd_ptr, ovf;
    uint8_t raw[6];
    uint8_t int_status;
    uint8_t num_samples = 0;
    uint32_t red_val, ir_val;

    // POINTERS INTERNOS (direcciones de memoria) DE ESCRITURA Y LECTURA
    MAX30102_ReadReg(dev, INT_STATUS_1, &int_status);
    MAX30102_ReadReg(dev, FIFO_WRITE_POINTER_REG, &wr_ptr);
    MAX30102_ReadReg(dev, FIFO_READ_POINTER_REG, &rd_ptr);
    MAX30102_ReadReg(dev, FIFO_OVERF_COUNTER_REG, &ovf);

    if (ovf > 0) {
    	num_samples = FIFO_DEPTH - 1;
    }
    else {
    	num_samples = (wr_ptr - rd_ptr + FIFO_DEPTH) % FIFO_DEPTH;
		if (num_samples == 0) return 0;
    }

    dev->samples_this_burst = 0;

    for (uint8_t i = 0; i < num_samples; i++)
    {
    	//	LECTURA DE MUESTRAS (6 Bytes = 3 RED + 3 IR)
        if (MAX30102_ReadBytes(dev, FIFO_DATA_REG, raw, 6) != MAX30102_OK)
            break;

        //	RECONSTRUCCION DE MUESTRAS POR SEPARADO (RED 0->2| IR 3->5)
        red_val = ((uint32_t)(raw[0] & 0x03) << 16) |
                  ((uint32_t)raw[1]           << 8)  |
                   (uint32_t)raw[2];

        ir_val  = ((uint32_t)(raw[3] & 0x03) << 16) |
                  ((uint32_t)raw[4]           << 8)  |
                   (uint32_t)raw[5];

        //	BUFFER CON HISTORIAL DE LECTURAS SEPARADAS
        dev->red_buf[dev->buf_idx] = red_val;
        dev->ir_buf[dev->buf_idx]  = ir_val;

        //	ÚLTIMO VALOR LEÍDO
        dev->last_red = red_val;
        dev->last_ir  = ir_val;

        //	CONTADOR PARA LLENAR BUFFER Y CONFIRMAR FULL (Listo para calcular HR y SPO2)
        dev->buf_idx++;
        if (dev->buf_idx >= SPO2_WINDOW){
        	dev->buf_idx = 0;
        }
		// CONTADOR ABSOLUTO (nunca resetea)
		if (dev->sample_count < 0xFFFF)
			dev->sample_count++;

		if (dev->sample_count >= HR_WINDOW && !dev->hr_buf_ready)
			dev->hr_buf_ready = 1;

		if (dev->sample_count >= SPO2_WINDOW && !dev->spo2_buf_ready)
			dev->spo2_buf_ready = 1;

		dev->samples_this_burst++;
    }

    return dev->samples_this_burst;
}

/*===============================================================================
						DEBUG DE LECTURA DEL FIFO
================================================================================*/

void MAX30102_DebugPrint(MAX30102_HANDLE *dev)
{
    char dbg[128];

    // FASE 1: verifica que llegan muestras y los valores son razonables
    snprintf(dbg, sizeof(dbg),
        "[FIFO] burst=%u sample_count=%u idx=%u hr_rdy=%u spo2_rdy=%u\r\n",
        dev->samples_this_burst,
        dev->sample_count,
        dev->buf_idx,
        dev->hr_buf_ready,
        dev->spo2_buf_ready);
    ITM_SendString(dbg);

    // FASE 2: verifica calidad de señal raw
    snprintf(dbg, sizeof(dbg),
        "[RAW]  IR=%lu | RED=%lu\r\n",
        dev->last_ir,
        dev->last_red);
    ITM_SendString(dbg);
}

/*===============================================================================
							DETECCIÓN DE DEDO
================================================================================*/

uint8_t MAX30102_FingerPresent(MAX30102_HANDLE *dev)
{
    return (dev->last_ir > 50000 && dev->last_red > 50000) ? 1 : 0;
}

uint32_t MAX30102_GetIR(MAX30102_HANDLE *dev)  { return dev->last_ir;  }
uint32_t MAX30102_GetRed(MAX30102_HANDLE *dev) { return dev->last_red; }

/*===============================================================================
							VALIDACIÓN DE SEÑAL
================================================================================*/

uint8_t MAX30102_SignalValid_HR(MAX30102_HANDLE *dev)
{
    if (!MAX30102_FingerPresent(dev))  return 0;
    if (!dev->hr_buf_ready)            return 0;
    if (dev->last_ir  > 260000)        return 0;
    if (dev->last_red > 260000)        return 0;
    return 1;
}

uint8_t MAX30102_SignalValid_SPO2(MAX30102_HANDLE *dev)
{
    if (!MAX30102_SignalValid_HR(dev))	return 0;
    if (!dev->spo2_buf_ready)			return 0;
    return 1;
}

/*===============================================================================
							PARÁMETROS DE LA SEÑAL
================================================================================*/

//	PROMEDIO DE LA SEÑAL/BUFFER (VALOR DC)
static float array_mean(uint32_t *arr, uint16_t len)
{
    uint64_t sum = 0;
    for (uint16_t i = 0; i < len; i++) sum += arr[i];
    return (float)sum / (float)len;
}

//	VALOR MAXIMO DE LA SEÑAL (PICO SISTÓLICO)
static uint32_t array_max(uint32_t *arr, uint16_t len)
{
    uint32_t m = arr[0];
    for (uint16_t i = 1; i < len; i++) if (arr[i] > m) m = arr[i];
    return m;
}

//	VALOR MINIMO DE LA SEÑAL (VALLE DIASTÓLICO)
static uint32_t array_min(uint32_t *arr, uint16_t len)
{
    uint32_t m = arr[0];
    for (uint16_t i = 1; i < len; i++) if (arr[i] < m) m = arr[i];
    return m;
}

/*===============================================================================
						REMOVER DC DE LA SEÑAL AC
================================================================================*/

// COMPONENTE AC: Resta el DC a cada muestra y devuelve array de floats
static void array_remove_dc(uint32_t *arr, uint16_t len,
                             float dc, float *out)
{
    for (uint16_t i = 0; i < len; i++)
        out[i] = (float)arr[i] - dc;
}

/*===============================================================================
								CALCULO DE HR
================================================================================*/
#define MIN_PEAK_DIST   10   // Muestras minimas entre picos

void MAX30102_Calc_HR(MAX30102_HANDLE *dev)
{
    uint16_t n = HR_WINDOW;

    //	REORDENAR BUFFER CIRCULAR CRONOLOGICAMENTE
    uint32_t ir_ordered[SPO2_WINDOW];
    uint16_t start = dev->buf_idx;

    for (uint16_t i = 0; i < n; i++)
        ir_ordered[i] = dev->ir_buf[(start + i) % SPO2_WINDOW];

    //	VALOR DC DE IR
    float dc_ir = array_mean(ir_ordered, n);

    //	AMPLITUD PICO-PICO PARA UMBRAL DINÁMICO
	float max_ir = (float)array_max(ir_ordered, n);
	float min_ir = (float)array_min(ir_ordered, n);
	float ac_pk2pk = max_ir - min_ir;

	//	CHECK: señal mínima para continuar
	if (ac_pk2pk < 500.0f)
	{
		dev->hr_valid   = 0;
		dev->heart_rate = 0;
		return;
	}

    //	AC: señal centrada en cero
    float ac_ir[HR_WINDOW];
    array_remove_dc(ir_ordered, n, dc_ir, ac_ir);

	//	UMBRALES SOBRE SEÑAL AC
	float threshold_high =  ac_pk2pk * 0.20f;
	float threshold_low  = -ac_pk2pk * 0.20f;

	// DETECCIÓN DE PICOS
	int     peak_count   = 0;
	int     first_peak   = -1;
	int     last_peak    = -1;
	int     last_peak_pos = -1;
	uint8_t above        = 0;
	for (uint16_t i = 1; i < n; i++)
	    {
	        if (!above && ac_ir[i] > threshold_high)
	        {
	            above = 1;
	            if (last_peak_pos < 0 || (i - last_peak_pos) >= MIN_PEAK_DIST)
	            {
	                peak_count++;
	                if (first_peak < 0) first_peak = i;
	                last_peak     = i;
	                last_peak_pos = i;
	            }
	        }
	        else if (above && ac_ir[i] < threshold_low)
	        {
	            above = 0;
	        }
	    }

	//	CALCULAR HR SI HAY SUFICIENTES PICOS DETECTADOS
	if (peak_count >= 4 && last_peak > first_peak)
	{
		int samples_between = last_peak - first_peak;
		int intervals       = peak_count - 1;

		int32_t hr_calc = (int32_t)(((int64_t)intervals * EFFECTIVE_SR * 60) / samples_between);

		// RANGO FISIOLÓGICO VÁLIDO
		if (hr_calc >= 45 && hr_calc <= 180)
		{
			dev->heart_rate = hr_calc;
			dev->hr_block_sum += hr_calc;
			dev->hr_block_count++;

			char hr[64];
			snprintf(hr, sizeof(hr), "HR: %ld BPM\r\n", dev->heart_rate);
			//ITM_SendString(hr);
			//BT_Send(hr);

			    //	CUANDO SE OBTIENEN 4 RESULTADOS
			    if (dev->hr_block_count >= 4)
			    {
			        //	PROMEDIO SIMPLE DE LOS 4 VALORES DEL BLOQUE
			        int32_t block_avg = dev->hr_block_sum / dev->hr_block_count;

			        //	SUAVIZADO ENTRE BLOQUES CONSECUTIVOS
			        if (dev->hr_valid == 0)
			            dev->hr_display = (float)block_avg;
			        else
			            dev->hr_display = 0.6f * dev->hr_display + 0.4f * (float)block_avg;

			        dev->heart_rate     = (int32_t)(dev->hr_display + 0.5f);
			        dev->hr_valid       = 1;

			        // RESET PARA EL SIGUIENTE BLOQUE (valores completamente nuevos)
			        dev->hr_block_sum   = 0;
			        dev->hr_block_count = 0;

					char hr_avg[64];
			        snprintf(hr_avg, sizeof(hr_avg), "HR: %ld BPM\r\n", dev->heart_rate);
					ITM_SendString(hr_avg);
					BT_Send(hr_avg);

			    }
		}
	}
}

/*===============================================================================
								CALCULO DE SPO2
================================================================================*/

void MAX30102_Calc_SPO2(MAX30102_HANDLE *dev)
{
	uint16_t n = SPO2_WINDOW;

    //	REORDENAR BUFFERS CIRCULARES CRONOLÓGICAMENTE
    uint32_t ir_ordered[SPO2_WINDOW];
    uint32_t red_ordered[SPO2_WINDOW];
    uint16_t start = dev->buf_idx;

    for (uint16_t i = 0; i < n; i++)
    {
        ir_ordered[i]  = dev->ir_buf[(start + i)  % SPO2_WINDOW];
        red_ordered[i] = dev->red_buf[(start + i) % SPO2_WINDOW];
    }

    //	DC: PROMEDIO DE CADA CANAL
    float dc_ir  = array_mean(ir_ordered,  n);
    float dc_red = array_mean(red_ordered, n);

    //	AC PICO-PICO: Amplitud de cada canal
    float ac_ir_pk2pk  = (float)array_max(ir_ordered,  n) - (float)array_min(ir_ordered,  n);
    float ac_red_pk2pk = (float)array_max(red_ordered, n) - (float)array_min(red_ordered, n);

    //	FILTRO: Señal minima para continuar
    if (dc_ir < 5000.0f || dc_red < 5000.0f)
    {
        dev->spo2_valid = 0;
        dev->spo2       = 0;
        return;
    }

    if (ac_ir_pk2pk < 500.0f || ac_red_pk2pk < 500.0f)
    {
        dev->spo2_valid = 0;
        dev->spo2       = 0;
        return;
    }

    //	FILTRO: AC demasiado alta (rechazar artefacto de movimiento)
    if (ac_ir_pk2pk > dc_ir * 0.5f || ac_red_pk2pk > dc_red * 0.5f)
    {
        dev->spo2_valid = 0;
        dev->spo2       = 0;
        ITM_SendString("[SPO2_DBG] Descartado: artefacto AC > 50% DC\r\n");
        return;
    }

    //	AC RMS
    float ac_ir[SPO2_WINDOW];
    float ac_red[SPO2_WINDOW];
    array_remove_dc(ir_ordered,  n, dc_ir,  ac_ir);
    array_remove_dc(red_ordered, n, dc_red, ac_red);

    float sum_ir = 0.0f, sum_red = 0.0f;
    for (uint16_t i = 0; i < n; i++)
    {
        sum_ir  += ac_ir[i]  * ac_ir[i];
        sum_red += ac_red[i] * ac_red[i];
    }

    float rms_ir  = sqrtf(sum_ir  / n);
    float rms_red = sqrtf(sum_red / n);

    //	CALCULAR RATIO R
    if (rms_ir < 1.0f || dc_red < 1.0f)
    {
        dev->spo2_valid = 0;
        dev->spo2 = 0;
        return;
    }

    float R = (rms_red / dc_red) / (rms_ir / dc_ir);

    //	CURVA EMPÍRICA OFICIAL (MAX30102): SpO2 = 1.5958422 * R² − 34.6596622·R + 112.6898759
    float spo2_f = 1.5958422f  * R * R
                 - 34.6596622f * R
                 + 112.6898759f;

    int32_t spo2_calc = (int32_t)(spo2_f + 0.5f);

    //	RANGO FISIOLÓGICO VÁLIDO
    if (spo2_calc >= 80 && spo2_calc <= 100)
    {
        dev->spo2       = spo2_calc;
        dev->spo2_valid = 1;
		dev->spo2_block_sum   += spo2_calc;
		dev->spo2_block_count++;

		char spo2[64];
        snprintf(spo2, sizeof(spo2), "SPO2: %ld %%\r\n", dev->spo2);
        //ITM_SendString(spo2);

	//	PROMEDIO DE 4 DATOS POR BLOQUE
    if (dev->spo2_block_count >= 4)
    {
    	dev->spo2_display     = (float)dev->spo2_block_sum / dev->spo2_block_count;
		dev->spo2             = (int32_t)(dev->spo2_display + 0.5f);
		dev->spo2_block_sum   = 0;
		dev->spo2_block_count = 0;

		char spo2_avg[64];
		snprintf(spo2_avg, sizeof(spo2_avg), "SPO2: %ld %%\r\n",
						dev->spo2);
		ITM_SendString(spo2_avg);
		BT_Send(spo2_avg);
    }
    }
    else
    {
    	dev->spo2_valid       = 0;
        dev->spo2             = 0;
        dev->spo2_block_sum   = 0;
        dev->spo2_block_count = 0;
    }
}
