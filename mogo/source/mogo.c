/*
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    mogo.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL02Z4.h"
#include "fsl_debug_console.h"

#include "sdk_hal_gpio.h"
#include "sdk_hal_uart0.h"
#include "sdk_hal_i2c0.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */
#define MMA851_I2C_DEVICE_ADDRESS 0x1D

#define MMA8451_WHO_AM_I_MEMORY_ADDRESS		0x0D
//DEFINICION DE CADA UNO DE LOS EJES DEL MAS SIGNIFICATIVO AL MENOS SIGNIFICATIVO
#define OUT_X_MSB_EJE	0x01 //MSB Y
#define OUT_X_LSB_EJE	0x02 //LSB Y
#define OUT_Y_MSB_EJE	0x03 //MSB X
#define OUT_Y_LSB_EJE	0x04 //LSB X
#define OUT_Z_MSB_EJE	0x05 //MSB Z
#define OUT_Z_LSB_EJE	0x06 //lSB Z

#define CTRL_REG1 	0x2A

/*
 * @brief   Application entry point.
 */
int main(void) {
	status_t status;
	status_t status_acel;
	uint8_t nuevo_byte_uart;
	uint8_t	nuevo_dato_i2c;
	//variables de x
	uint16_t out_x_h;
	uint16_t out_x_l;
	uint16_t out_x;
	//variables de y
	uint16_t out_y_h;
	uint16_t out_y_l;
	uint16_t out_y;
	//variables de z
	uint16_t out_z_h;
	uint16_t out_z_l;
	uint16_t out_z;

	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();
#endif

	(void)uart0Inicializar(115200);	//115200bps
	(void)i2c0MasterInit(100000);	//100kbps
	status_acel=i2c0MasterWriteByte(MMA851_I2C_DEVICE_ADDRESS, 0x2A,1); //0x2a control reg poner arriba el define

	PRINTF("Pulse las teclas x,y,z para obtener la coordenada de dicha variable\n");


	while(1) {

		if(uart0CuantosDatosHayEnBuffer()>0){
			status=uart0LeerByteDesdeBuffer(&nuevo_byte_uart);
			if(status==kStatus_Success){
				printf("dato:%c\r\n",nuevo_byte_uart);
				switch (nuevo_byte_uart) {

				break;
				case 'M':
					i2c0MasterReadByte(&nuevo_dato_i2c, MMA851_I2C_DEVICE_ADDRESS, MMA8451_WHO_AM_I_MEMORY_ADDRESS);

					if(nuevo_dato_i2c==0x1A)
						printf("MMA8451 encontrado!!\r\n");
					else
						printf("MMA8451 error\r\n");

					break;

				case 'X':
				case 'x':
					i2c0MasterReadByte(&out_x_h, MMA851_I2C_DEVICE_ADDRESS, OUT_X_MSB_EJE);
					i2c0MasterReadByte(&out_x_l, MMA851_I2C_DEVICE_ADDRESS, OUT_X_LSB_EJE);

					out_x_h<<=8;
					out_x= out_x_h|out_x_l;
					out_x>>=2;
					printf("Dato eje X: %d\r\n",out_x);
					gpioPutHigh(KPTB10);
					gpioPutLow(KPTB6);
					gpioPutHigh(KPTB7);

					break;

				case 'Y':
				case 'y':
					i2c0MasterReadByte(&out_y_h, MMA851_I2C_DEVICE_ADDRESS, OUT_Y_MSB_EJE);
					i2c0MasterReadByte(&out_y_l, MMA851_I2C_DEVICE_ADDRESS, OUT_Y_LSB_EJE);

					out_y_h<<=8;
					out_y= out_y_h|out_y_l;
					out_y>>=2;
					printf("Dato Eje Y: %d\r\n",out_y);
					gpioPutHigh(KPTB10);
					gpioPutLow(KPTB7);
					gpioPutHigh(KPTB6);

					break;

				case 'Z':
				case 'z':
					i2c0MasterReadByte(&out_z_h, MMA851_I2C_DEVICE_ADDRESS, OUT_Z_MSB_EJE);
					i2c0MasterReadByte(&out_z_l, MMA851_I2C_DEVICE_ADDRESS, OUT_Z_LSB_EJE);

					out_z_h<<=8;
					out_z= out_z_h|out_z_l;
					out_z>>=2;
					printf("Dato eje Z: %d\r\n",out_z);
					gpioPutHigh(KPTB7);
					gpioPutLow(KPTB10);
					gpioPutHigh(KPTB6);


				}
			}else{
				printf("Variable no identificada\r\n");
			}
		}
	}

	return 0 ;
}

