/*
<<<<<<< Updated upstream
Ejercicio 3. Aplicación de respuesta a las lecturas del sensor. Tablas de look-up.

La curva obtenida para los sensores IR en el apartado anterior tiene una expresión bastante
compleja, y con diferente pendiente según el tramo. Debido a la poca potencia matemática de la
mayoría de los micros, no se suele programar una función matemática que relacione Voltaje y
distancia; en su lugar se suele optar por tabular las distancias de interés y asociar dichas distancias
– o intervalos de las mismas – a las lecturas correspondientes obtenidas en el ADC.
Programa una aplicación para la placa TIVA conectada a un sensor de distancia tipo SHARP,
establece 3 posibles intervalos activos equiespaciados de funcionamiento en el rango del sensor
que hayas medido, y programa una aplicación que realice las siguientes acciones:
	• Si se detecta una distancia en el primer intervalo activo [x1,x2]cm, se deberá encender el
	LED verde.
	• Si se detecta una distancia en el segundo intervalo activo [x2,x3]cm, se deberá encender el
	LED rojo
	• Si se detecta una distancia en el tercer intervalo [x3-x4cm], se deberán encender ambos
	LEDS.
	• Para cualquier otra distancia (x<x1, o x > x4) los LEDS deben permanecer apagados.
	• Indica claramente en tu programa a que valores en centimetros se corresponden esos
	intervalos.
*/

/*
 * Idealmente, se deberían usar interrupciones para "avisar" de que las muestran están disponibles, y recogerlas.
 * Además, es más adecuado usar un timer para disparar periodicamente el muestreo, ya que de lo contrario se ocupa mucho tiempo de CPU.
=======
 * Idealmente, se deber�an usar interrupciones para "avisar" de que las muestran est�n disponibles, y recogerlas.
 * Adem�s, es m�s adecuado usar un timer para disparar periodicamente el muestreo, ya que de lo contrario se ocupa mucho tiempo de CPU.
>>>>>>> Stashed changes
 *
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_adc.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include <math.h>
#include "inc/tm4c123gh6pm.h"  /* Incluir cabecera del dispositivo para asegurar que las macros INT_* se resuelvan al habilitar interrupciones */

/* Configuración para sobremuestreo */
#define OVERSAMPLE_COUNT 16 /* Cambia este valor para más/menos sobremuestreo */
/* Umbral (cm) alrededor del extremo cercano (4 cm) para considerarlo fuera de rango */
#define DIST_TOL_CM 1.0f

/* Caracterización del sensor SHARP (distancia en cm y voltaje en V) */
#define NUM_SHARP_POINTS 14
static const float sharp_dist_cm[NUM_SHARP_POINTS] = {4,6,8,10,12,14,16,18,20,22,24,26,28,30};
static const float sharp_volt_v[NUM_SHARP_POINTS] = {3.05f,1.81f,1.41f,1.28f,1.11f,0.95f,0.83f,0.67f,0.61f,0.56f,0.51f,0.46f,0.42f,0.37f};

/* Distancias tabuladas como enteros para usar binary_lookup (cm) */
static const unsigned short sharp_dist_us[NUM_SHARP_POINTS] = {4,6,8,10,12,14,16,18,20,22,24,26,28,30};

/* Variables compartidas con la ISR */
volatile uint64_t g_adc_sum = 0;
volatile uint32_t g_adc_count = 0;

int main(void)
{
	/* Declarar variables al inicio de la función para compatibilidad con C89 */
	float minD, maxD, width;

	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ); // Reloj del sistema a 40MHz

	/* Habilitar GPIOE para el pin analógico AIN0 (PE3) y configurar como ADC */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3); /* AIN0 en PE3 */

	/* Habilitar ADC0 */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));

	ADCSequenceDisable(ADC0_BASE, 1);
	HWREG(ADC0_BASE + ADC_O_PC) = (ADC_PC_SR_125K); /* Tasa de muestreo */
	/* Disparo por instrucciones del procesador */
	ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
	/* Configuramos el secuenciador 1 con un solo paso que lea AIN0 */
	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 1);

	/* Habilitar la interrupción del ADC1 (secuencia 1) */
	ADCIntEnable(ADC0_BASE, 1);
	IntEnable(INT_ADC0SS1);
	IntMasterEnable();

	/* Configurar LEDs en el puerto F: PF3 = verde, PF1 = rojo */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_3);

	/* Cálculo de límites de los 3 intervalos equiespaciados en el rango medido */
	minD = sharp_dist_cm[0];
	maxD = sharp_dist_cm[NUM_SHARP_POINTS-1];
	width = (maxD - minD) / 3.0f;

	/* Bucle principal: lanzamos conversiones periódicamente y la ISR hace sobremuestreo y el procesamiento */
	while(1)
	{
<<<<<<< Updated upstream
		/* Iniciar una conversión; la ISR la recogerá */
		ADCProcessorTrigger(ADC0_BASE, 1);
		/* Espera corta: control de tasa de muestreo (ajusta si es necesario) */
		SysCtlDelay(SysCtlClockGet() / 2000); /* ~0.5ms aproximado */
=======
   	  ADCIntClear(ADC0_BASE, 1); // Limpia el flag de interrupcion del ADC
	  // Dispara una nueva secuencia de conversiones
    	  ADCProcessorTrigger(ADC0_BASE, 1);
	  // Espera a que finalice la conversion (esto no har�a falta si se utilizasen interrupciones, ya que la interrrupci�n se
       // produce cuando finaliza la conversi�n)
        while(!ADCIntStatus(ADC0_BASE, 1, false))
	  {}
   	  // Tras haber finalizado la conversion, leemos los datos del secuenciador a un array
    	  ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
    	  // Calculamos la temperatura media como la media de las muestras de los 4 conversores
    	  ui32TempAvg = (ui32ADC0Value[0] + ui32ADC0Value[1] + ui32ADC0Value[2] + ui32ADC0Value[3] + 2)/4;
    	  // Y lo convertimos a grados centigrados y Farenheit, usando la formula indicada en el Data Sheet
	  ui32TempValueC = (1475 - ((2475 * ui32TempAvg)) / 4096)/10;
	  ui32TempValueF = ((ui32TempValueC * 9) + 160) / 5;
>>>>>>> Stashed changes
	}
}

/* ISR del ADC0 secuencia 1: recoge cada muestra, acumula y cuando llega a OVERSAMPLE_COUNT
   calcula promedio, mapea a distancia y enciende LEDs. */
void ADC0Seq1Handler(void)
{
	uint32_t value;
	/* Limpiar la interrupción del ADC */
	ADCIntClear(ADC0_BASE, 1);
	/* Leer la muestra */
	ADCSequenceDataGet(ADC0_BASE, 1, &value);

	g_adc_sum += value;
	g_adc_count++;

	if (g_adc_count >= OVERSAMPLE_COUNT) {
		uint32_t avg = (uint32_t)(g_adc_sum / OVERSAMPLE_COUNT);
		float voltage = ((float)avg * 3.3f) / 4095.0f;

		  /* Calcular una estimación inicial de distancia a partir de la tensión media.
			  El flujo que seguimos para usar 'binary_lookup' es:
			  1) Convertir la media ADC a voltaje (variable 'voltage' arriba).
			  2) Buscar en la tabla de voltajes el índice cuyo voltaje característico
				  es el más cercano al medido; tomamos su distancia como estimación.
			  3) Redondear esa distancia a centímetros y usar 'binary_lookup' sobre
				  la tabla de distancias enteras (sharp_dist_us) para obtener el índice
				  correspondiente en la tabla.
			  4) Comparar el índice obtenido con el anterior (idx-1) y quedarnos con
				  el que realmente dé la distancia más cercana.
		  */

		/* Estimación inicial de distancia basada en el voltaje medido */
		  float estimated_distance_cm;
		  int volt_best = 0;
		  float volt_best_diff = fabsf(voltage - sharp_volt_v[0]);
		  int vi;
		  for (vi = 1; vi < NUM_SHARP_POINTS; vi++) {
				float dd = fabsf(voltage - sharp_volt_v[vi]);
				if (dd < volt_best_diff) { volt_best_diff = dd; volt_best = vi; }
		  }
		  estimated_distance_cm = sharp_dist_cm[volt_best];

		/* Detectar si la medición está fuera del rango caracterizado.
		   Para el extremo lejano (mayor que maxD) ya funcionaba correctamente
		   por comparación de tensión; para el extremo cercano la curva del
		   sensor puede comportarse de forma no lineal, por eso hacemos una
		   comprobación basada en la distancia estimada: si la distancia
		   estimada está dentro de DIST_TOL_CM del mínimo medido (4 cm) la
		   consideramos fuera de rango y apagamos LEDs. */
		bool out_of_range = false;
		  /* Comparación por distancia: si la estimación cae en el margen cercano
			  al mínimo de la tabla lo tratamos como fuera de rango. */
		  if (estimated_distance_cm <= (sharp_dist_cm[0] + DIST_TOL_CM)) {
			out_of_range = true; /* demasiado cerca, consideramos fuera de rango */
		}
		/* Comprobación opcional para extremo lejano por tensión (ligera tolerancia) */
		const float V_TOL = 0.02f;
		if (voltage < sharp_volt_v[NUM_SHARP_POINTS - 1] - V_TOL) {
			out_of_range = true; /* demasiado lejos */
		}

		  /* Redondear la distancia estimada y buscar el índice en la tabla entera */
		  unsigned short key = (unsigned short)(estimated_distance_cm + 0.5f);
		  unsigned short idx = binary_lookup((unsigned short *)sharp_dist_us, key, 0, NUM_SHARP_POINTS - 1);
		  int best_idx = idx;
		  /* binary_lookup devuelve la primera posición con A[idx] >= key; comprobamos
			  el anterior para elegir el más cercano en valor absoluto */
		  if (idx > 0) {
				float d_hi = fabsf(estimated_distance_cm - sharp_dist_cm[idx]);
				float d_lo = fabsf(estimated_distance_cm - sharp_dist_cm[idx - 1]);
				if (d_lo < d_hi) best_idx = idx - 1;
		  }
		  /* Distancia final utilizada para la lógica de los LEDs */
		  float measured_distance = sharp_dist_cm[best_idx];

	/* Determinar intervalos equiespaciados */
	const float minD = sharp_dist_cm[0];
		const float maxD = sharp_dist_cm[NUM_SHARP_POINTS-1];
		const float width = (maxD - minD) / 3.0f;
		const float x1 = minD;
		const float x2 = minD + width;
		const float x3 = minD + 2.0f * width;
		const float x4 = maxD;

		/* Encender/apagar LEDs según intervalo. Si está fuera del rango medido,
		   apagamos todos los LEDs. */
		if (out_of_range) {
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_1, 0);
		} else if (measured_distance >= x1 && measured_distance < x2) {
			/* Verde */
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_1, GPIO_PIN_3);
		} else if (measured_distance >= x2 && measured_distance < x3) {
			/* Rojo */
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_1, GPIO_PIN_1);
		} else if (measured_distance >= x3 && measured_distance <= x4) {
			/* Ambos */
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_1, GPIO_PIN_3 | GPIO_PIN_1);
		} else {
			/* Apagados */
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_1, 0);
		}

		/* Reset acumuladores */
		g_adc_sum = 0;
		g_adc_count = 0;
	}
}


unsigned short binary_lookup(unsigned short *A, unsigned short key, unsigned short imin, unsigned short imax)
{
  unsigned int imid;

  while (imin < imax)
    {
      imid= (imin+imax)>>1;
 
      if (A[imid] < key)
        imin = imid + 1;
      else
        imax = imid;
    }
    return imax;    //Al final imax=imin y en dicha posicion hay un numero mayor o igual que el buscado
}



//Añadidas para que no de fallo al linkear
void GPIOFIntHandler(void){}
void GPIOEIntHandler(void){}
void Timer0A_Handler(void){}
