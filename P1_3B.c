/* ************************************************************************** */
/* UNIVERSIDAD DE MALAGA               DEPARTAMENTO DE TECNOLOGIA ELECTRONICA */
/* http://www.uma.es                                    http://www.dte.uma.es */
/* ========================================================================== */
/* PROGRAMA :  PWM-Servo                                                      */
/* VERSION  : 1.0                                                             */
/* TARGET   : Kit  TIVA Launchpad IDE CCSv                                    */
/*
   DESCRIPCIÓN:
    Genera dos ondas PWM a 50Hz (20ms) por los pines PF2 y PF3.
    La PWM por PF3 se pondrá (de momento) con un ciclo de trabajo fijo de 
	1.5ms (parado)
    La PWM por PF2 se inicia con un ciclo de trabajo de 1.5ms (STOPCOUNT, 
	servo parado o prácticamente parado)
    Al pulsar los botones el ciclo aumenta/disminuye en CYCLE_ INCREMENTS 
    Los márgenes teóricos del servo están entre 1ms-2m (COUNT_1MS-COUNT_2MS)

    Si nada mas empezar el servo esta prácticamente parado (o se para con 
	2-3 pulsaciones de uno u otro botón), se
    considera calibrado. Si no, se tendrá que abrir y calibrar mediante el 
	potenciómetro interno.

  **************************************************************************   */
#include <stdint.h>
#include <stdbool.h>
// Librerias que se incluyen tipicamente para configuracion de perifericos y pinout
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h" 
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
// Libreria de control del sistema
#include "driverlib/sysctl.h"
// Incluir librerias de periférico y otros que se vaya a usar para control PWM y gestión
// de botones  (TODO)

#include "driverlib/gpio.h" // Libreria GPIO
#include "driverlib/pwm.h" // Libreria PWM
#include "drivers/buttons.h" // Liberia botones TIVA Launchpad
#include "driverlib/interrupt.h" // Libreria Interrupciones
#include "inc/hw_ints.h" // Definiciones de interrupciones


#define PERIOD_PWM 12499 // (625KHz/50 Hz) -1	// TODO: Ciclos de reloj para conseguir una señal periódica de 50Hz (según reloj de periférico usado)
                            // PERIOD_PWM se obtendría como (pwm_clk / 50) - 1);  donde 50 es la frecuencia a obtener; y pwm_clk es la frecuencia de reloj del modulo PWM/Timer
							// (se determina con SysCtlClockSet y  SysCtlPWMClockSet en el generador PWM; y SysCtlClockSet y las funciones de preescalado en los timers)
#define COUNT_1MS 624   // TODO: Ciclos en el ciclo de trabajo para amplitud de pulso de 1ms (max velocidad en un sentido)
#define STOPCOUNT 949 // (625KHz *1.52ms) -1 = 949 // TODO: Ciclos en el ciclo de trabajo  para amplitud de pulso de parada (1.52ms)
#define COUNT_2MS 1249   // TODO: Ciclos en el ciclo de trabajo  para amplitud de pulso de 2ms (max velocidad en el otro sentido)

// Los valores anteriores COUNT_XMS se obtendrían como (pwm_clk * valor_en_ms_del_pulso/1000);
// Todos los valores anteriores tienen que ser de 16BITS máximo

#define NUM_STEPS 50    // Pasos/pulsaciones de boton necesarios para variar entre el pulso de 2ms al de 1ms (elegir a gusto del usuario)
#define CYCLE_INCREMENTS (abs(COUNT_1MS-COUNT_2MS))/NUM_STEPS  // Variacion de amplitud del ciclo de trabajo tras pulsacion del boton

#define PWM_BASE_FREQ 50 // Esta es la frecuencia de la señal PWM que queremos generar; en este caso concreto son 50Hz

uint32_t ui32DutyCycle1; //Variable que alamcenaran el ciclo de trabajo(entre 1 y 2 ms)
uint32_t ui32DutyCycle2; //Variable que alamcenaran el ciclo de trabajo(entre 1 y 2 ms)

int main(void){
    // uint32_t ui32Period; // Variable que alamcenaran el numero de ciclos de reloj para el periodo(20ms)
    // uint32_t pwm_clk;
    ui32DutyCycle1 = (COUNT_2MS + STOPCOUNT)/2; // Inicializo el ciclo de trabajo al estado de reposo del servo.
    ui32DutyCycle2 = (STOPCOUNT + COUNT_1MS)/2;

    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // Reloj del sistema a 40MHz;     // Elegir reloj adecuado para los valores de ciclos sean de tamaño soportable (cantidades menores de 16bits). Max frecuencia: 80MHz

  // Configura pulsadores placa TIVA (int. por flanco de bajada)
    ButtonsInit();
        GPIOIntTypeSet(GPIO_PORTF_BASE, ALL_BUTTONS,GPIO_BOTH_EDGES);
        GPIOIntEnable(GPIO_PORTF_BASE,ALL_BUTTONS);
        IntEnable(INT_GPIOF);

  // Configuracion  ondas PWM: frecuencia 50Hz, anchura inicial= valor STOPCOUNT, 1520us para salida por PF2, y COUNT_STOP para salida por PF3(o puedes poner otro valor si quieres que se mueva)
  	// Opcion 1: Usar un Timer en modo PWM (ojo! Los timers PWM solo soportan cuentas 
    //  de 16 bits, a menos que uséis un prescaler/timer extension)
  	// Opcion 2: Usar un módulo PWM(no dado en Sist. Empotrados pero mas sencillo)
    //  SysCtlPWMClockSet(???);


    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1); //Habilita modulo PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);    // Habilita puerto salida para señal PWM (ver en documentacion que pin se corresponde a cada módulo PWM)

    // Rueda 1
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinConfigure(GPIO_PF2_M1PWM6);

    // Rueda 2
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);

       // pwm_clk = SysCtlClockGet() / 64;  // 64, ya que es el factor de división del reloj de PWM respecto al reloj del sistema(SYSCTL_PWMDIV_64)
                                             // Si hubiesemos escogido otro SYSCTL_PWMDIV_XX, dividiríamos entre XX en vez de 64
           //val_load = (pwm_clk /PWM_BASE_FREQ  ) - 1; //IGUAL QUE PERIOD_PWM   //Valor del contador para la freq. pwm. En este caso --> 625Khz/50 = 6250-1: este valor NUNCA puede ser mayor que 0xFFFF!!
                                                       // Si te sale mayor, deberias usar un factor de división mayor (SYSCTL_PWMDIV_XX), o un reloj del sistema mas lento (SysCtlClockSet)

           PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);   // Módulo PWM contara hacia abajo
           PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, PERIOD_PWM);    // Carga la cuenta que establece la frecuencia de la señal PWM

           PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui32DutyCycle1);  // Establece el ciclo de trabajo R1
           PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, ui32DutyCycle2);  // Establece el ciclo de trabajo R2

           PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true); // Habilita la salida de la señal R1
           PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true); // Habilita la salida de la señal R2

           PWMGenEnable(PWM1_BASE, PWM_GEN_3); //Habilita/pone en marcha el generador PWM



  // Codigo principal, (poner en bucle infinito o bajo consumo)
           while(1)
           {
           }
}


// Rutinas de interrupción de pulsadores
// Boton Izquierdo: modifica  ciclo de trabajo en CYCLE_INCREMENTS para el servo conectado a PF2, hasta llegar a  COUNT_1MS
// Boton Derecho: modifica  ciclo de trabajo en CYCLE_INCREMENTS para el servo conectado a PF2, hasta llegar a COUNT_2MS

void GPIOFIntHandler(void)
{
    int32_t i32Status = GPIOIntStatus(GPIO_PORTF_BASE,ALL_BUTTONS);
    // Boton Izquierdo: reduce ciclo de trabajo en CYCLE_INCREMENTS para el servo conectado a PF4, hasta llegar a MINCOUNT
    if(((i32Status & LEFT_BUTTON) == LEFT_BUTTON)){
        if(ui32DutyCycle1 > STOPCOUNT)
        {
            ui32DutyCycle1 -= CYCLE_INCREMENTS;
            if(ui32DutyCycle1 < STOPCOUNT)
            {
                ui32DutyCycle1 = STOPCOUNT;
            }
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui32DutyCycle1);
        }
    }
    // Boton Derecho: aumenta ciclo de trabajo en CYCLE_INCREMENTS para el servo conectado a PF4, hasta llegar a MAXCOUNT
    if(((i32Status & RIGHT_BUTTON) == RIGHT_BUTTON)){ //Se ha pulsado boton derecho
        if(ui32DutyCycle1 < COUNT_2MS) //El ciclo de trabajo aun no esta en el maximo
        {
            ui32DutyCycle1 += CYCLE_INCREMENTS;
            if(ui32DutyCycle1 > COUNT_2MS) //Si el incremento hace que el ciclo supere el max le asignamos el max.
            {
                ui32DutyCycle1 = COUNT_2MS;
            }
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui32DutyCycle1); //Imponemos el nuevo ciclo de trabajo.
        }
    }
    GPIOIntClear(GPIO_PORTF_BASE,ALL_BUTTONS);  //limpiamos flags
}
