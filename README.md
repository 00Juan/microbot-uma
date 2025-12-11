# 🤖 SumoBOT Firmware (Tiva C Series)

### Project Team (Architects)

- **Alejandro Moyano Crespillo (AleSMC)**
- **Juan Estevez Delgado**
- **Pedro Lopez**

---

> **Platform:** Tiva C TM4C123GH6PM (ARM Cortex-M4F)  
> **Context:** University of Malaga (UMA) - Dept. Tecnología Electrónica  
> **License:** MIT / Academic Use

![Status](https://img.shields.io/badge/Status-Development-yellow)
![Language](https://img.shields.io/badge/Language-C99-blue)
![Architecture](https://img.shields.io/badge/Architecture-Layered_HAL-green)

## 📋 Resumen del Proyecto

Firmware de alto rendimiento para un robot de sumo autónomo (categoría Mini-Sumo). El sistema está diseñado sobre una arquitectura modular no bloqueante, priorizando la determinabilidad y la baja latencia en la respuesta de los sensores.

El núcleo del sistema opera a **40 MHz** y gestiona la adquisición de datos mediante **ADC Hardware Sequencers** para liberar la CPU para la toma de decisiones estratégicas.

---

## 🔌 Arquitectura de Hardware (Wiring)

**IMPORTANTE:** Toda la configuración de pines está centralizada en `include/RobotConfig.h`. No hardcodear pines en los drivers.

[Image of Tiva C Series TM4C123GH6PM Pinout Diagram]

### 1. Actuadores (Motores)

Se utilizan servos de rotación continua (Futaba modificados/trucados).

- **Frecuencia PWM:** 50 Hz (Periodo 20ms).
- **Driver:** PWM1 Generador 3.

| Función             | Pin Tiva | Periférico | Detalles            |
| :------------------ | :------- | :--------- | :------------------ |
| **Motor Izquierdo** | `PF2`    | M1PWM6     | Control diferencial |
| **Motor Derecho**   | `PF3`    | M1PWM7     | Control diferencial |

### 2. Sensores (Percepción)

La lectura de sensores prioriza la velocidad.

| Sensor                | Pin Tiva | Tipo       | Hardware    | Notas                                |
| :-------------------- | :------- | :--------- | :---------- | :----------------------------------- |
| **Sharp (Distancia)** | `PE2`    | Analógico  | ADC0 (SS1)  | Detecta enemigo (rango ~20-80cm)     |
| **CNY70 (Línea)**     | `PE1`    | Analógico  | ADC0 (SS1)  | Detecta borde del dohyo (Blanco)     |
| **Bumper Izquierdo**  | `PA2`    | Digital In | GPIO Port A | Pull-Up Interno Activo (GND = Press) |
| **Bumper Derecho**    | `PA3`    | Digital In | GPIO Port A | Pull-Up Interno Activo (GND = Press) |

### 3. Interfaz (Debug)

| Componente     | Pin Tiva | Estado        |
| :------------- | :------- | :------------ |
| **LED Estado** | `PF1`    | Rojo (On/Off) |

---

## 📂 Estructura del Proyecto

El proyecto sigue una estructura de capas (Layered Architecture) para desacoplar el hardware de la lógica.

```text
SumoBOT_Project/
├── include/                # API PÚBLICA (Headers .h)
│   ├── RobotConfig.h       # [CRÍTICO] Mapeo de pines, constantes y calibración.
│   ├── motor.h             # Interfaz de control de movimiento.
│   ├── sensors.h           # Interfaz de lectura del entorno.
│   └── strategy.h          # (Futuro) Máquina de estados.
│
├── src/                    # IMPLEMENTACIÓN (Source .c)
│   ├── motor.c             # Driver PWM/GPIO para servos.
│   ├── sensors.c           # Driver ADC/GPIO para sensores.
│   └── strategy.c          # Lógica de combate.
│
├── main.c                  # Scheduler y Setup principal.
├── tm4c123gh6pm_startup_ccs.c # Vector Table
```

---

## ⚙️ Configuración y Calibración

Para garantizar un rendimiento óptimo en el tatami, es necesario ajustar las constantes en `include/RobotConfig.h` según el hardware específico de cada unidad.

### 1. Calibración de Motores (Punto Muerto)

Los servos trucados pueden tener una ligera deriva. Si al iniciar el robot (comando `Motor_Stop`) las ruedas giran lentamente:

1.  Observar qué rueda gira.
2.  Ajustar `PWM_STOP_TICKS` (Valor base: 949 para 1.5ms).
    - Incrementar o decrementar en pasos de 10 unidades hasta que el motor se detenga completamente.

### 2. Umbrales de Sensores

- **Sensor de Distancia (Sharp):**

  - Basado en la curva de voltaje: `20cm ~= 0.61V`.
  - La constante `SHARP_THRESHOLD_TICKS` define la distancia de ataque.
  - _Ajuste:_ Si el robot ataca "fantasmas", aumentar el valor del umbral (reducir sensibilidad).

- **Sensor de Línea (CNY70):**
  - Detecta el borde blanco del Dohyo.
  - **Procedimiento de Calibración:**
    1.  Leer valor ADC sobre Negro (Tatami).
    2.  Leer valor ADC sobre Blanco (Borde).
    3.  Configurar `LINE_THRESHOLD_TICKS` en el punto medio.
  - _Nota:_ Prioridad P0 (Survive) en la Máquina de Estados.

---

## 🛡️ Normativa (Mini-Sumo)

Este firmware está diseñado para cumplir estrictamente con la normativa estándar (LNRC v1.1) descrita en la documentación oficial:

### Reglas de Combate (Software Constraints)

- **Inicio (Start Delay):** El robot debe esperar **5 segundos** obligatorios tras la activación antes de desplegar cualquier mecanismo o movimiento (Art. 5).
- **Autonomía:** El funcionamiento debe ser 100% autónomo. Prohibido el control remoto (Art. 4).
- **Parada:** El robot debe detenerse si el árbitro lo indica o si finaliza el tiempo de asalto.

### Especificaciones de la Categoría

- **Dimensiones:** 10cm x 10cm (Sin límite de altura).
- **Peso Máximo:** 500g.
- **Dohyo:** Círculo de 90cm de diámetro (Negro) con borde de 5cm (Blanco).

---

## 🔮 Roadmap v2.0 (Capacidad de Expansión)

La arquitectura actual (`src/sensors.c`) y el hardware del ADC (Secuenciador SS1) han sido diseñados dejando **2 ranuras libres** para futuras mejoras tácticas sin necesidad de refactorizar el código base.

La alimentación de estos sensores es despreciable (<50mA) comparada con los motores, por lo que no requiere rediseño de la etapa de potencia.

### Propuesta de Mejora A: Retaguardia (Defensa)

Evita que el robot salga del dohyo al retroceder o ser empujado.

- **Hardware:** Sensor Reflexivo (CNY70 / QRE1113).
- **Conexión:** Pin **PE3** (Canal AIN0).
- **Lógica:** Prioridad P0 (Survive) -> Si detecta blanco atrás -> Avance Rápido (Escape).

### Propuesta de Mejora B: Visión Estéreo (Ataque)

Elimina la "visión de túnel" del Sharp central único. Dos sensores angulados permiten diferenciar si el enemigo escapa por la izquierda o derecha.

- **Hardware:** Sensor Distancia Sharp (GP2Y0A21).
- **Conexión:** Pin **PE0** (Canal AIN3).
- **Lógica:** \* Solo Izquierda: Giro suave Izq.
  - Solo Derecha: Giro suave Der.
  - Ambos: Ataque Turbo (Ramming).

---
