#include <stdint.h>
#include <stdbool.h>
// Librerias que se incluyen tipicamente para configuracion de perifericos y pinout
#include "inc/tm4c123gh6pm.h" // Definiciones específicas del micro (INT_*, etc.)
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
// Libreria de control del sistema
#include "driverlib/sysctl.h"
// Incluir librerias de perif�rico y otros que se vaya a usar para control PWM y gesti�n
// de botones  (TODO)

#include "driverlib/gpio.h" // Libreria GPIO
#include "driverlib/pwm.h" // Libreria PWM
#include "drivers/buttons.h" // Liberia botones TIVA Launchpad
#include "driverlib/interrupt.h" // Libreria Interrupciones
#include "driverlib/timer.h" // Libreria Timer (TimerDisable, TimerConfigure, etc.)


void GPIOFIntHandler();
void GPIOEIntHandler();
void Timer0A_Handler();
void ADC0Seq1Handler();


#define DEBOUNCE_MS 20 // Tiempo de debounce en ms para antirrebotes del sensor de contacto

#define PERIOD_PWM 12499 // (625KHz/50 Hz) -1   // TODO: Ciclos de reloj para conseguir una se�al peri�dica de 50Hz (seg�n reloj de perif�rico usado)
                            // PERIOD_PWM se obtendr�a como (pwm_clk / 50) - 1);  donde 50 es la frecuencia a obtener; y pwm_clk es la frecuencia de reloj del modulo PWM/Timer
                            // (se determina con SysCtlClockSet y  SysCtlPWMClockSet en el generador PWM; y SysCtlClockSet y las funciones de preescalado en los timers)
#define COUNT_1MS 624   // TODO: Ciclos en el ciclo de trabajo para amplitud de pulso de 1ms (max velocidad en un sentido)
#define STOPCOUNT 949 // (625KHz *1.52ms) -1 = 949 // TODO: Ciclos en el ciclo de trabajo  para amplitud de pulso de parada (1.52ms)
#define COUNT_2MS 1249   // TODO: Ciclos en el ciclo de trabajo  para amplitud de pulso de 2ms (max velocidad en el otro sentido)

// Los valores anteriores COUNT_XMS se obtendr�an como (pwm_clk * valor_en_ms_del_pulso/1000);
// Todos los valores anteriores tienen que ser de 16BITS m�ximo

#define NUM_STEPS 50    // Pasos/pulsaciones de boton necesarios para variar entre el pulso de 2ms al de 1ms (elegir a gusto del usuario)
#define CYCLE_INCREMENTS (abs(COUNT_1MS-COUNT_2MS))/NUM_STEPS  // Variacion de amplitud del ciclo de trabajo tras pulsacion del boton

#define PWM_BASE_FREQ 50 // Esta es la frecuencia de la se�al PWM que queremos generar; en este caso concreto son 50Hz

uint32_t ui32DutyCycle1; //Variable que alamcenaran el ciclo de trabajo(entre 1 y 2 ms)
uint32_t ui32DutyCycle2; //Variable que alamcenaran el ciclo de trabajo(entre 1 y 2 ms)

int main(void){
    // uint32_t ui32Period; // Variable que alamcenaran el numero de ciclos de reloj para el periodo(20ms)
    // uint32_t pwm_clk;
    ui32DutyCycle1 = COUNT_2MS; // Inicializo el servo maxima velocidad hacia delante.
    ui32DutyCycle2 = COUNT_1MS;

  // Configura el GPIO del sensor de contacto)
    /* Habilitar periferico GPIOE y esperar listo */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //Habilita GPIO E
    //. while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    /* Configurar PE3 como entrada y con pull-up débil (si el sensor a tierra) */
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_3);
    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    /* Si el sensor da 3.3V cuando detecta, usar WPD (pull-down) en su lugar */

    /* Limpiar cualquier flag pendiente */
    GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_3);

    /* Configurar tipo de interrupción como flanco de bajada */
    GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_FALLING_EDGE);

    /* Habilitar interrupción del pin en el módulo GPIO */
    GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_3);

    /* Habilitar interrupción en el NVIC */
    IntEnable(INT_GPIOE);

    /* Habilitar interrupciones globales (muy importante) */
    IntMasterEnable();  /* o IntGlobalEnable() dependiendo de la versión de driverlib */


    /* Habilitar Timer0 */
SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));

TimerDisable(TIMER0_BASE, TIMER_A);
TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT);
{
    uint32_t ticks = (SysCtlClockGet() / 1000) * DEBOUNCE_MS; /* número de ticks para DEBOUNCE_MS */
    TimerLoadSet(TIMER0_BASE, TIMER_A, ticks);
}
TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
IntEnable(INT_TIMER0A);
/* No arrancamos el timer aún; lo iniciamos desde la ISR GPIO */



  // Configuracion  ondas PWM: frecuencia 50Hz, anchura inicial= valor STOPCOUNT, 1520us para salida por PF2, y COUNT_STOP para salida por PF3(o puedes poner otro valor si quieres que se mueva)
    // Opcion 1: Usar un Timer en modo PWM (ojo! Los timers PWM solo soportan cuentas
    //  de 16 bits, a menos que us�is un prescaler/timer extension)
    // Opcion 2: Usar un m�dulo PWM(no dado en Sist. Empotrados pero mas sencillo)

    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // Reloj del sistema a 40MHz;     // Elegir reloj adecuado para los valores de ciclos sean de tama�o soportable (cantidades menores de 16bits). Max frecuencia: 80MHz
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64); // Establece reloj del generador PWM (40MHz/64=625KHz)

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1); //Habilita modulo PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);    // Habilita puerto salida para se�al PWM (ver en documentacion que pin se corresponde a cada m�dulo PWM)

    // Rueda 1
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinConfigure(GPIO_PF2_M1PWM6);

    // Rueda 2
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);

           PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);   // M�dulo PWM contara hacia abajo
           PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, PERIOD_PWM);    // Carga la cuenta que establece la frecuencia de la se�al PWM

           PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui32DutyCycle1);  // Establece el ciclo de trabajo R1
           PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, ui32DutyCycle2);  // Establece el ciclo de trabajo R2

           PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true); // Habilita la salida de la se�al R1
           PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true); // Habilita la salida de la se�al R2

           PWMGenEnable(PWM1_BASE, PWM_GEN_3); //Habilita/pone en marcha el generador PWM



  // Codigo principal, (poner en bucle infinito o bajo consumo)
           while(1)
           {
           }
}

// Rutinas de interrupci�n de sensor de contacto
void GPIOEIntHandler(void)
{
    int32_t i32Status = GPIOIntStatus(GPIO_PORTE_BASE, GPIO_PIN_3);
    // Sensor en PE_3:Cambia la direccion del bot
    if(((i32Status & GPIO_PIN_3) == GPIO_PIN_3))
    {
        /* Deshabilitar la interrupción del pin para ignorar rebotes */
        GPIOIntDisable(GPIO_PORTE_BASE, GPIO_PIN_3);

        /* Limpiar flag del pin */
        GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_3);

        /* Reiniciar y arrancar el timer one-shot para debounce */
        TimerDisable(TIMER0_BASE, TIMER_A);
        TimerLoadSet(TIMER0_BASE, TIMER_A, (SysCtlClockGet() / 1000) * DEBOUNCE_MS);
        TimerEnable(TIMER0_BASE, TIMER_A);
    }
    GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_3);  //limpiamos flags
}

// Antirrebotes del sensor de contacto
void Timer0A_Handler(void)
{
    /* Leer estado real del pin */
    /* Leer el nivel lógico del pin (GPIOPinRead devuelve 0 si está a GND) */
    uint32_t pinLevel = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_3);

    /* Si el sensor está conectado a GND cuando se pulsa (activo-low), entonces
       consideramos pulsado cuando pinLevel == 0 (el bit correspondiente está a 0). */
    if ((pinLevel & GPIO_PIN_3) == 0) {
    
        // Retroceder 10cm, girar 90º y voler a avanzar.
        // Deberiamos llamar a las funciones de movimiento de nuestra libreria.

        // CODIGO APROXIMADO
        if(ui32DutyCycle1 == COUNT_1MS)
        {
            ui32DutyCycle1 = COUNT_2MS; // Avanzo
            ui32DutyCycle2 = COUNT_1MS;
        }
        else
        {
            ui32DutyCycle1 = COUNT_1MS; // Retrocedo
            ui32DutyCycle2 = COUNT_2MS;
        }
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui32DutyCycle1);  // Establece el ciclo de trabajo R1
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, ui32DutyCycle2);  // Establece el ciclo de trabajo R2
    }
    /* Rehabilitar la interrupción GPIO para futuros eventos */
    GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_3); /* limpiar por si hay rastro */
    GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_3);

    /* Limpiar la interrupción del timer */
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    
}



void ADC0Seq1Handler(void){}
void GPIOFIntHandler(void){}



