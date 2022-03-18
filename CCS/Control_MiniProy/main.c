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
/* Este Programa realiza lecturas de una MPU6050, con el fin de procesar el ángulo
 * calculado con respecto al eje "y" y así mediante un controladro PID estabilizar
 * un balancín mecánico. De igual Forma la información es enviada por medio de
 * comunicación UART para poder visualizar la información de forma gráfica y
 * numérica, tanto de los datos leídos de la MPU6050 como del ángulo del carril.
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
#include "driverlib/adc.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include <math.h>
#include "utils/uartstdio.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/ssi.h"



//-------------------------------------
// Definición de Variables
//-------------------------------------
#define NUM_SPI_DATA    1  //Palabras a enviar
#define SPI_FREC  4000000  //Freq
#define SPI_ANCHO      16



//Valores PID
volatile float rk, yk, e_k_1, e_k, Ek, eD,u_k, kP, kI, kD, u_k2;

int             clockFreq; //Device freq

volatile bool   g_bMPU6050Done;
tI2CMInstance   g_sI2CMSimpleInst;
tMPU6050        sMPU6050;

// Definición de Arrays para valores en bruto de la MPU6050
float  fAccel[3];
float  fGyro[3];

//Variables para almacenar valores
volatile float x = 0, y = 0, z = 0;
volatile float gx = 0, gy = 0, gz = 0;
int x1,y1,z1,x2,y2,z2,ay;



//Valores envío SPI
uint16_t data;
uint16_t val;
uint32_t g_pui32residual[NUM_SPI_DATA];
uint16_t g_freq_muestreo = 1000;

//-------------------------------------
// Definición de Interrupciones
//-------------------------------------
void
Timer0IntHandler(void)
{
    uint32_t pui32DataTx[NUM_SPI_DATA];
    uint8_t  ui32Index;

    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //Valor deseado de alcanzar
    rk = 0.000;
    //Valor de ángulo obtenido en MPU6050
    yk = ay;

    //Cálculos Controlador PID
    e_k = rk - yk;
    eD = e_k - e_k_1;
    Ek = Ek + e_k;
    u_k = kP*e_k + kI*Ek + kD*eD;
    e_k_1 = e_k;

    //Límites de Voltajes (Circuito de Acoplamiento)
    if (u_k > 6.46)
        u_k = 6.46;
    else if (u_k < -6.46)
        u_k = -6.46;

    //Mapeo de datos
    u_k2 = (u_k*341.25 + 2047.5);

    //Envío SPI
    val = (uint16_t) (u_k2);

    data = (0b0111000000000000 | val);

    pui32DataTx[0] = (uint32_t)(data);

    for(ui32Index = 0; ui32Index < NUM_SPI_DATA; ui32Index++)
    {
        SSIDataPut(SSI0_BASE, pui32DataTx[ui32Index]);
    }

    while(SSIBusy(SSI0_BASE))
    {
    }
}

//Interrupción I2C

void I2CMSimpleIntHandler(void)
{
    I2CMIntHandler(&g_sI2CMSimpleInst);
}

//-------------------------------------
// Definición de Interrupciones
//-------------------------------------
void mpu6050config(void);
void mpu6050callback(void *pvCallbackData, uint_fast8_t ui8Status);
void init_i2c0(void);
void configure_uart(void);
void config_adc(void);
void enable_timer_ssi(void);

//-------------------------------------
// MAIN
//-------------------------------------
int main(void){
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_OSC_INT
                   | SYSCTL_XTAL_16MHZ);
    //Config
    configure_uart();
    init_i2c0();
    mpu6050config();
    config_adc();
    enable_timer_ssi();

    //Valores iniciales
    data = 0b0111000000000000;
    val = 0b0000000000000000;
    e_k_1 = 0;
    Ek = 0;

    //Constantes del PID
    kP =5;
    kI = 0;
    kD =2;

    // Ciclo While para que la programación se ejecute constantemente
    while (1)
    {

            g_bMPU6050Done = false;
            MPU6050DataRead(&sMPU6050, mpu6050callback, &sMPU6050);
            while (!g_bMPU6050Done)
            {
            }
            // Lectura MPU6050
            MPU6050DataAccelGetFloat(&sMPU6050, &fAccel[0],
                                     &fAccel[1], &fAccel[2]);
            MPU6050DataGyroGetFloat(&sMPU6050, &fGyro[0],
                                    &fGyro[1], &fGyro[2]);

            // Se asignan los valores obtenidos a variables para poder manipular
            x2 = fGyro[0];
            y2 = fGyro[1];
            z2 = fGyro[2];
            x1=fAccel[0];
            y1=fAccel[1];
            z1=fAccel[2];

            ay = ((atan2(fAccel[0], fAccel[1])*180.0)/3.14)-90; //Ángulo balancín


            //Envío de Datos por UART
            UARTprintf("%d %d %d %d %d %d %d \n", (int)x1, (int)y1, (int)z1, (int)x2,(int)y2,(int)z,(int)ay);
    }
}

//Config MPU6050 (Extraído de librerías de TivaWare)
void mpu6050config(void)
{
    // Initialize the MPU6050.
        g_bMPU6050Done = false;
        MPU6050Init(&sMPU6050, &g_sI2CMSimpleInst, 0x68,
                    mpu6050callback, &sMPU6050);
        while (!g_bMPU6050Done)
        {
        }
        // Configure the MPU6050 for +/- 4 g accelerometer range.

        g_bMPU6050Done = false;
        MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_ACCEL_CONFIG,
                               ~MPU6050_ACCEL_CONFIG_AFS_SEL_M,
            MPU6050_ACCEL_CONFIG_AFS_SEL_4G, mpu6050callback, &sMPU6050);
        while (!g_bMPU6050Done)
        {
        }


        g_bMPU6050Done = false;
        MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_PWR_MGMT_1, 0x00,
                               0b00000010 & MPU6050_PWR_MGMT_1_DEVICE_RESET,
                               mpu6050callback, &sMPU6050);
        while (!g_bMPU6050Done)
        {
        }

        g_bMPU6050Done = false;
        MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_PWR_MGMT_2,
                               0x00, 0x00, mpu6050callback, &sMPU6050);
        while (!g_bMPU6050Done)
        {
        }
}

/* The function that is provided by this example as a callback when MPU6050
    transactions have completed.
 */
//Función que "llama" a la mpu, en base al ejemplo de Texas instruments
void
mpu6050callback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    // See if an error occurred.
    if (ui8Status != I2CM_STATUS_SUCCESS)
    {
        // An error occurred, so handle it here if required.
    }
    // Indicate that the MPU6050 transaction has completed.
    g_bMPU6050Done = true;
}

//Config I2C
void init_i2c0(void)
{
    //Habilita el modulo 0 de la i2c
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //resetea el modulo
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.
    // I2C data transfer rate set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;

    // Initialize the I2C master driver.
    I2CMInit(&g_sI2CMSimpleInst, I2C0_BASE, INT_I2C0, 0xff,
             0xff, SysCtlClockGet());

}

//Config UART
void configure_uart(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    IntMasterEnable();
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000);
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}


void config_adc(void)
{
    // The ADC0 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    // Step 0 en la secuencia 2: Canal 0 (ADC_CTL_CH0) en modo single-ended.
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH0);

    ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH1
                             | ADC_CTL_IE | ADC_CTL_END);

    // Since sample sequence 2 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 2);  // Notar el cambio de "secuencia".

    // Clear the interrupt status flag.  This is done to make sure the
    // interrupt flag is cleared before we sample.
    ADCIntClear(ADC0_BASE, 2);

    // Enable the peripherals used by this example.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    // The SSI0 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                           SSI_MODE_MASTER, SPI_FREC, SPI_ANCHO);

    // Enable the SSI0 module.
    SSIEnable(SSI0_BASE);

    while(SSIDataGetNonBlocking(SSI0_BASE, &g_pui32residual[0]))
        {
        }
}

//Se habilita el TMR
void enable_timer_ssi(void)
{
    // Enable processor interrupts.
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
        IntMasterEnable();
        TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
        // Configure the two 32-bit periodic timers.
        TimerLoadSet(TIMER0_BASE, TIMER_A,
                     (uint32_t)(SysCtlClockGet()/g_freq_muestreo));
        // Setup the interrupt for the timer timeout.
        IntEnable(INT_TIMER0A);
        TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
        // Enable the timers.
        TimerEnable(TIMER0_BASE, TIMER_A);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        GPIOPinConfigure(GPIO_PA2_SSI0CLK);
        GPIOPinConfigure(GPIO_PA3_SSI0FSS);
        GPIOPinConfigure(GPIO_PA4_SSI0RX);
        GPIOPinConfigure(GPIO_PA5_SSI0TX);
        GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4
                       | GPIO_PIN_3 | GPIO_PIN_2);
        SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                                 SSI_MODE_MASTER, SPI_FREC, SPI_ANCHO);
        SSIEnable(SSI0_BASE);
        while(SSIDataGetNonBlocking(SSI0_BASE, &g_pui32residual[0])){}


}


