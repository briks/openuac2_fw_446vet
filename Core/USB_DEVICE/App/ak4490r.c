#include "ak4490r.h"
#include "usbd_audio_if.h"
#include "main.h"

extern I2C_HandleTypeDef AK4490R_I2C_HANDLE;

static uint8_t play;
uint8_t registre;
uint8_t regread;

AUDIO_CodecTypeDef ak4490r_instance =
{
	AK4490R_Init,
	NULL,
	AK4490R_Play,
	AK4490R_SetFormat,
	AK4490R_Stop,
	NULL,
	AK4490R_SetMute,
	AK4490R_SetVolume
};

static AK4490R_RegisterTypeDef reg;

uint8_t AK4490R_Init()
{
	LL_GPIO_ResetOutputPin(PDN_GPIO_Port, PDN_Pin);
	LL_mDelay(10);
	LL_GPIO_SetOutputPin(PDN_GPIO_Port, PDN_Pin);
	LL_mDelay(100);
	//reg1 input selection: set 32bits data default and i2s 1100 set to i2s input (no auto detect) 0000
	//reg7 filter bw and system mute: set the mute b10000001 or normal b10000000
	//reg8: set gpio1 to spdif input & gpio2 to whatever analog input for shutdown d13d8
	//reg11: set wich input use when decoding SPDIF data GPIO1: d3d0
	//reg27 general configuration: set ch1 volume on to share volume between ch1 and ch2 b11011100

	//shut down the oscillator b11110000
	registre = 0xf0;
	//HAL_I2C_Mem_Write_IT(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&registre, sizeof(registre));

	//reg14 normal operation b10001010 
	registre = 0x8a;
	HAL_I2C_Mem_Read(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, 0x0e, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&regread, sizeof(regread), HAL_MAX_DELAY );
	HAL_I2C_Mem_Write(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, 0x0e, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&registre, sizeof(registre), HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, 0x0e, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&regread, sizeof(regread), HAL_MAX_DELAY );

	//reg1 input selection: set 32bits data default and i2s 1100 set to i2s input (no auto detect) 0000
	registre = 0xcc;
	//HAL_I2C_Mem_Write(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, 0x01, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&registre, sizeof(registre), HAL_MAX_DELAY);

	//reg8: set gpio1 to spdif input & gpio2 to whatever analog input for shutdown d13d8
	registre = 0xd8;
	//HAL_I2C_Mem_Write_IT(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, 0x08, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&registre, sizeof(registre));

	//reg11: set wich input use when decoding SPDIF data GPIO1: d3d0
	registre = 0x30;
	//HAL_I2C_Mem_Write_IT(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, 0x0b, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&registre, sizeof(registre));

	//reg27 try +18db gain (not good idea) made volumne ch2 same a ch1
	registre = 0xdc;
	HAL_I2C_Mem_Write(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, 0x1b, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&registre, sizeof(registre), HAL_MAX_DELAY);
	return 0;
}

uint8_t AK4490R_SetVolume(uint8_t vol)
{
	if (AK4490R_I2C_HANDLE.State == HAL_I2C_STATE_READY)
	{
		vol = (vol > 0) ? (vol + 155) : 0;
		registre = vol;
		//registre = 0x0;
		HAL_I2C_Mem_Write(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_REG15_ADDR, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&registre, 1, HAL_MAX_DELAY);
		//HAL_I2C_Mem_Write(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_REG16_ADDR, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&registre, 1, HAL_MAX_DELAY);
		//HAL_I2C_Mem_Read(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_REG15_ADDR, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&registre, 1, HAL_MAX_DELAY );
	}

	return 0;
}

uint8_t AK4490R_SetMute(uint8_t mute)
{
	if (AK4490R_I2C_HANDLE.State == HAL_I2C_STATE_READY)
	{
		if (mute)
		{
			registre = 0x81;
		}
		else
		{
			registre = 0x80;
		}
		//reg7 filter bw and system mute: set the mute b10000001 or normal b10000000
		//HAL_I2C_Mem_Write_IT(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, 0x07, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&registre, 1);
	}

	return 0;
}

uint8_t AK4490R_SetFormat(uint8_t format)
{
	if (AK4490R_I2C_HANDLE.State == HAL_I2C_STATE_READY)
	{
		if (format == AUDIO_FORMAT_DSD)
		{
			reg.control3 |= AK4490R_DP;
		}
		else
		{
			reg.control3 &= ~AK4490R_DP;
		}

//		reg.control1 &= ~AK4490R_RSTN;
		//HAL_I2C_Mem_Write_IT(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_CONTROL3_ADDR, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&reg.control3, 1);
//		reg_reset = 1;
	}

	return 0;
}

uint8_t AK4490R_Play()
{
	play = 1;
	return 0;
}

uint8_t AK4490R_Stop()
{
	if (AK4490R_I2C_HANDLE.State == HAL_I2C_STATE_READY)
	{
		registre = 0x81;
	}

	//HAL_I2C_Mem_Write_IT(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, 0x07, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&registre, 1);
	return 0;
}

void AK4490R_ProcessEvents()
{
	if (play)
	{
		LL_mDelay(20);
		registre = 0x80;
		//while (HAL_I2C_Mem_Write(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, 0x07, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&registre, 1, 1000) != HAL_OK);
		play = 0;
	}
}


