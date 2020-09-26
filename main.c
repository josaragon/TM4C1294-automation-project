#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"

#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/i2c.h"
#include "driverlib/systick.h"
#include "driverlib/ssi.h"

#include "FT800_TIVA.h"


#include "utils/uartstdio.h"

#include "HAL_I2C.h"
#include "bme280.h"
#include "bmi160.h"
#include "OPT3001.h"
#include "tmp007.h"

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "FT800_TIVA.h"
#define dword long
#define byte char
// =======================================================================
// Function Declarations
// =======================================================================


int RELOJ,temp;
#define MSEC 40000 //Defino 1 milisegundo para el delay

void Timer0IntHandler(void);


char Cambia=0;

float lux;
char string[50];
int DevID=0;


//Variables Pantalla
char chipid = 0;                        // Holds value of Chip ID read from the FT800

unsigned long cmdBufferRd = 0x00000000;         // Store the value read from the REG_CMD_READ register
unsigned long cmdBufferWr = 0x00000000;         // Store the value read from the REG_CMD_WRITE register

uint8_t inicia[] = {0,0};

int Fin_Rx=0;
char Buffer_Rx;
unsigned long POSX, POSY, BufferXY;
unsigned long POSYANT=0;
unsigned int CMD_Offset = 0;
unsigned long REG_TT[6];
const unsigned long REG_CAL[6]={21959,177,4294145463,14,4294950369,16094853};

#define NUM_SSI_DATA            3



//Sensor Temperatura
int16_t T_amb, T_obj;

 float Tf_obj, Tf_amb;
 int lux_i, T_amb_i, T_obj_i;

 // BME280
 int returnRslt;
 int g_s32ActualTemp   = 0;
 unsigned int g_u32ActualPress  = 0;
 unsigned int g_u32ActualHumity = 0;
 struct bme280_t bme280;

 // BMI160/BMM150
 int8_t returnValue;
 struct bmi160_gyro_t        s_gyroXYZ;
 struct bmi160_accel_t       s_accelXYZ;
 struct bmi160_mag_xyz_s32_t s_magcompXYZ;


 //Calibration off-sets
 int8_t accel_off_x;
 int8_t accel_off_y;
 int8_t accel_off_z;
 int16_t gyro_off_x;
 int16_t gyro_off_y;
 int16_t gyro_off_z;
 float T_act,P_act,H_act;
 bool BME_on = true;

 int T_uncomp,T_comp;
char mode;
long int inicio, tiempo;
char texto[50],texto2[50];
int estado;

volatile long int ticks=0;

void IntTick(void){
    ticks++;
}
int main(void) {

//Configuracion Placa
    RELOJ=SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);
    Conf_Boosterpack(1, RELOJ); //Se inicializa el Sensors Boosterpack en el boosterpack 1
    HAL_Init_SPI(2, RELOJ);  //Se inicializa la pantalla en el boosterpack 2

//Configuracion Timers
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerClockSourceSet(TIMER0_BASE, TIMER_CLOCK_SYSTEM);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, RELOJ/4 -1);
    TimerIntRegister(TIMER0_BASE, TIMER_A,Timer0IntHandler);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntMasterEnable();
    TimerEnable(TIMER0_BASE, TIMER_A);

//Configuracion puertos GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

//Configuracion UART
    UARTStdioConfig(0, 115200, RELOJ);

//OPT3001
    UARTprintf("\033[2J \033[1;1H Inicializando OPT3001... ");
    OPT3001_init();
    UARTprintf("Hecho!\n");
    UARTprintf("Leyendo DevID... ");
    DevID=OPT3001_readDeviceId();
    UARTprintf("DevID= 0X%x \n", DevID);

//TMP007
    UARTprintf("Inicializando ahora el TMP007...");
    sensorTmp007Init();
    UARTprintf("Hecho! \nLeyendo DevID... ");
    DevID=sensorTmp007DevID();
    UARTprintf("DevID= 0X%x \n", DevID);
    sensorTmp007Enable(true);

//BME280
    UARTprintf("Inicializando BME280... ");
    bme280_data_readout_template();
    bme280_set_power_mode(BME280_NORMAL_MODE);
    UARTprintf("Hecho! \nLeyendo DevID... ");
    readI2C(BME280_I2C_ADDRESS2,BME280_CHIP_ID_REG, &DevID, 1);
    UARTprintf("DevID= 0X%x \n", DevID);

//BMI160:Acelerometro o giroscopo
    UARTprintf("Inicializando BMI160, modo NAVIGATION... ");
    bmi160_initialize_sensor();
    bmi160_config_running_mode(APPLICATION_NAVIGATION);
    //bmi160_accel_foc_trigger_xyz(0x03, 0x03, 0x01, &accel_off_x, &accel_off_y, &accel_off_z);
    //bmi160_set_foc_gyro_enable(0x01, &gyro_off_x, &gyro_off_y, &gyro_off_z);
    UARTprintf("Hecho! \nLeyendo DevID... ");
    readI2C(BMI160_I2C_ADDR2,BMI160_USER_CHIP_ID_ADDR, &DevID, 1);
    UARTprintf("DevID= 0X%x \n", DevID);

//
    SysTickIntRegister(IntTick);
    SysTickPeriodSet(12000);
    SysTickIntEnable();
    SysTickEnable();

//
    estado=0;

    switch(estado){

    //Pantalla bienvenida
    case 0:
        Inicia_pantalla();
        SysCtlDelay(RELOJ/3);

        //Pinto la pantalla
        Nueva_pantalla(0x10,0x10,0x10); //Se crea una nueva pantalla

        //Fondo
        Comando(CMD_BEGIN_RECTS);

        ComColor(244,183,231);  //  Rosa
        ComVertex2ff(0,0);
        ComVertex2ff(320,240);

        ComColor(255,255,255);  //Blanco
        ComVertex2ff(5,5);
        ComVertex2ff(315,235);

        ComColor(244,183,231);  //Rosa
        ComVertex2ff(8,8);
        ComVertex2ff(312,232);

        //Letras bienvenido

        strcpy(texto,"BIENVENIDO AL SISTEMA");
        strcpy(texto2,"DE AUTOMATIZACION :)");
        ComColor(29,49,238);
        ComTXT(160,75, 23, OPT_CENTERX, texto);
        ComTXT(160,125, 23, OPT_CENTERX, texto2);

        Dibuja(); //Dibujamos la pantalla
        SysCtlDelay(500*MSEC); //Refrescamos la pantalla 2 veces por segundo

        break;

    //Pantalla eleccion:abrir o crear
    case 1:
        Inicia_pantalla();
        SysCtlDelay(RELOJ/3);

        //Pinto la pantalla
        Nueva_pantalla(0x10,0x10,0x10); //Se crea una nueva pantalla

        //Fondo
        Comando(CMD_BEGIN_RECTS);

        ComColor(244,183,231);  //  Rosa
        ComVertex2ff(0,0);
        ComVertex2ff(320,240);

        ComColor(255,255,255);  //Blanco
        ComVertex2ff(5,5);
        ComVertex2ff(315,235);

        ComColor(244,183,231);  //Rosa
        ComVertex2ff(8,8);
        ComVertex2ff(312,232);


        ComColor(128,255,255);  //Verde azulado
        ComVertex2ff(20,20);
        ComVertex2ff(300,100);

        ComColor(128,255,255);  //Verde azulado
        ComVertex2ff(20,140);
        ComVertex2ff(300,220);

        Comando(CMD_END);




        Dibuja(); //Dibujamos la pantalla
        SysCtlDelay(500*MSEC); //Refrescamos la pantalla 2 veces por segundo

        break;
    }


















        while(1)
        {

        if(Cambia==1){

            Cambia=0;
            inicio=ticks;

            lux=OPT3001_getLux();
            lux_i=(int)round(lux);

            sensorTmp007Read(&T_amb, &T_obj);
            sensorTmp007Convert(T_amb, T_obj, &Tf_obj, &Tf_amb);
            T_amb_i=(short)round(Tf_amb);
            T_obj_i=(short)round(Tf_obj);

            returnRslt = bme280_read_pressure_temperature_humidity(
                    &g_u32ActualPress, &g_s32ActualTemp, &g_u32ActualHumity);
            T_act=(float)g_s32ActualTemp/100.0;
            P_act=(float)g_u32ActualPress/100.0;
            H_act=(float)g_u32ActualHumity/1000.0;

            bmi160_bmm150_mag_compensate_xyz(&s_magcompXYZ);
            bmi160_read_accel_xyz(&s_accelXYZ);
            bmi160_read_gyro_xyz(&s_gyroXYZ);
            tiempo=ticks;

            //Luminosidad
            UARTprintf("\033[10;1H---------------------------------------\n");
            sprintf(string,"  OPT3001: %.3f Lux\n",lux);
            UARTprintf(string);

            //Temperatura
            UARTprintf("---------------------------------------\n");
            sprintf(string,"  TMP007:  T_a:%.3f, T_o:%.3f \n", Tf_amb, Tf_obj);
            UARTprintf(string);

            //BME280
            UARTprintf("---------------------------------------\n");
            sprintf(string, "  BME: T:%.2f C  P:%.2fmbar  H:%.3f  \n",T_act,P_act,H_act);
            UARTprintf(string);

            //Magnetometro
            UARTprintf("---------------------------------------\n");
            sprintf(string, "  BMM:  X:%6d\033[17;22HY:%6d\033[17;35HZ:%6d  \n",s_magcompXYZ.x,s_magcompXYZ.y,s_magcompXYZ.z);
            UARTprintf(string);

            //Acelerometro
            UARTprintf("---------------------------------------\n");
            sprintf(string, "  ACCL: X:%6d\033[19;22HY:%6d\033[19;35HZ:%6d  \n",s_accelXYZ.x,s_accelXYZ.y,s_accelXYZ.z);
            UARTprintf(string);

            //Girosocopo
            UARTprintf("---------------------------------------\n");
            sprintf(string, "  GYRO: X:%6d\033[21;22HY:%6d\033[21;35HZ:%6d  \n",s_gyroXYZ.x,s_gyroXYZ.y,s_gyroXYZ.z);
            UARTprintf(string);

            //Tiempo
            UARTprintf("---------------------------------------\n");
            tiempo=(tiempo-inicio);
            sprintf(string, "TConv: %d (0.1ms)",tiempo);
            UARTprintf(string);




        }
    }



    return 0;
}


void Timer0IntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); //Borra flag
    Cambia=1;
}
