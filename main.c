#include "include/RobotConfig.h"
#include "include/motor.h"
#include "include/timer.h"
#include "include/odometry.h"

// No incluimos strategy.h para no interferir
// #include "include/strategy.h" 

// Declaración forward de las funciones de odometría (o crea un odometry.h)
void Odometry_Init(void);
void mover_robot(float cm);
void girar_robot(float deg);

int main(void) {
    // Setup
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    Motor_Init();
    Timer_Init();
    Odometry_Init();

    // Prueba de la Práctica 2.2
    
    // 1. Avanzar 30 cm
    mover_robot(30.0);
    Timer_WaitMillis(1000);

    // 2. Girar 90 grados a la derecha
    girar_robot(90.0);
    Timer_WaitMillis(1000);
    
    // 3. Volver (Girar 180 y avanzar 30)
    girar_robot(180.0);
    Timer_WaitMillis(500);
    mover_robot(30.0);

    while(1) {
        // Fin de la prueba
    }
}