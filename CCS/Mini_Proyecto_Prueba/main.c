/* Universidad del Valle de Guatemala
 * Diseño e Innovación en Ingeniería 1
 * Mini Proyecto
 *
 *
 * Katharine Senn
 * Carné 18012
 *
 * Librerías extraídas de: https://github.com/mathmagson/mpu6050_tm4c123g
 * Autor: mathmagson
 *
 * Edición por: Katharine Senn
 */

//________________________________________________________________________________
//________________________________________________________________________________
//________________________________________________________________________________


//-------------------------------------
// Se incluyen las Librerías necesarias
//-------------------------------------
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/hw_mpu6050.h"
#include "sensorlib/mpu6050.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"

#include <math.h>

#include "utils/uartstdio.h"
#include "driverlib/uart.h"

//-------------------------------------
// Definición de Variables
//-------------------------------------
volatile bool g_bMPU6050Done;

tI2CMInstance g_sI2CMSimpleInst;

int clockFreq;    // Freq.


//-------------------------------------
// Definición de Funciones
//-------------------------------------


// Inicialización Comunicación I2C

void InitI2C0(void)
{
    //Habilitar Módulo 0 del I2C
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //Resetear Módulo
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //Habilitar periféricos (GPIO)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configuración de pines (PB2 y PB3)
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);


    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Inicialización del módulo
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

    //Limpieza de FIFO
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;

    // Inicializar Driver I2C Master
    I2CMInit(&g_sI2CMSimpleInst, I2C0_BASE, INT_I2C0, 0xff, 0xff, SysCtlClockGet());

}

// Configuración UART

void ConfigureUART(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000); //BaudRate en 115200 igual el módulo COM
}

void delayMS(int ms) {
    SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ;
}



// Indicador en forma de "Callback" que indica que la MPU6050 finalizó su inicializació  sin errores
void MPU6050Callback(void *pvCallbackData, uint_fast8_t ui8Status)
{

    if (ui8Status != I2CM_STATUS_SUCCESS)  //Revisión de Errores
    {
        //
        // Espcio (opcional) para corregir o manejar los errores encontrados
        //
    }
    //
    // Levantar bandera de inicialización de MPU5060 exitosa
    //
    g_bMPU6050Done = true;
}


// IntHandler para I2C
void I2CMSimpleIntHandler(void)
{
    //
    // Llamar a la interrupción del I2C
    //
    I2CMIntHandler(&g_sI2CMSimpleInst);
}

// -------------------------------------------------------------------------------------------
// The MPU6050 example. // Función en donde se realiza la lectura de datos y el envío por UART
// -------------------------------------------------------------------------------------------
void MPU6050Example(void)
{
    float fAccel[3], fGyro[3];  // Definición de Arrays para valores en bruto de la MPU6050
    tMPU6050 sMPU6050;
    float x1 = 0, y1 = 0, z1 = 0, x2 = 0, y2 = 0, z2 = 0; //Variables para almacenar valores en bruto y calculados
    float ay = 0;
    int contador = 0;

    //Inicialización MPU6050
    g_bMPU6050Done = false;
    MPU6050Init(&sMPU6050, &g_sI2CMSimpleInst, 0x68, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }

    //
    // Configuración de rango acelerómetro (MPU60509)
    //
    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_ACCEL_CONFIG, ~MPU6050_ACCEL_CONFIG_AFS_SEL_M,
        MPU6050_ACCEL_CONFIG_AFS_SEL_4G, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }


    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_PWR_MGMT_1, 0x00, 0b00000010 & MPU6050_PWR_MGMT_1_DEVICE_RESET, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }

    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_PWR_MGMT_2, 0x00, 0x00, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }


    // -------------------------------------------------------------------------------------------
    // Ciclo While para leer constantemente los datos deseados
    // -------------------------------------------------------------------------------------------

    while (1)
    {
       contador = contador +1; //Contador para dar un período de tiempo entre cada lectura del giroscopio

        //
        // Solicitud de Lectura
        //
        g_bMPU6050Done = false;
        MPU6050DataRead(&sMPU6050, MPU6050Callback, &sMPU6050);
        while (!g_bMPU6050Done)
        {
        }
        //
        // Actualización de datos
        //
        MPU6050DataAccelGetFloat(&sMPU6050, &fAccel[0], &fAccel[1], &fAccel[2]);
        MPU6050DataGyroGetFloat(&sMPU6050, &fGyro[0], &fGyro[1], &fGyro[2]);


        //
        // Almacenamiento de datos en nuevas variables
        //
        x1 = fGyro[0]; //Datos en brito
        y1 = fGyro[1];
        z1 = fGyro[2];

        x2 = fAccel[0];
        y2 = fAccel[1];
        z2 = fAccel[2];

        ay = ((atan2(fAccel[0], fAccel[1])*180.0)/3.14)-90; //Ángulo balancín

        // -----------------------------------------------------
        // Envío de Datos MPU6050 por medio de Comunicación UART
        // -----------------------------------------------------

        //Se cuenta hasta 50 antes de realizar el envío para realizar el mismo únicamente al rededor de 20 veces por segundo
        if (contador == 50){
            UARTprintf("%d %d %d %d %d %d %d\n", (int)x1, (int)y1, (int)z1, (int)x2, (int)y2, (int)z2, (int)ay);
            contador = 0;
        }


    }
}


// -------------------------------------------------------------------------------------------
// Programa Principal
// -------------------------------------------------------------------------------------------
int main()
{

    // LLamar funciones a ejecutar

    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_OSC_INT | SYSCTL_XTAL_16MHZ);

    InitI2C0();
    ConfigureUART();
    MPU6050Example();
    return(0);
}





