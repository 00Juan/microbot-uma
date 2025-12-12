# 🤖 SumoBOT Firmware (Tiva C Series)

### Project Team

- **Alejandro Moyano Crespillo (AleSMC)**
- **Juan Estevez Delgado**
- **Pedro Lopez**

> **Platform:** Tiva C TM4C123GH6PM (ARM Cortex-M4F)
> **Context:** University of Malaga (UMA) - Dept. Tecnología Electrónica

---

# 🎓 Módulo Académico: Navegación Diferencial (Práctica 2.2)

> **⚠️ BRANCH SPECIFIC:** Esta documentación pertenece exclusivamente a la rama `practice/double-encoder`.
> El código aquí contenido difiere del firmware de competición (Main Branch).

### 📝 Descripción

Implementación de las rutinas de navegación determinista exigidas en la Práctica 2.2 (Robótica UMA). Este firmware permite mover el robot distancias exactas y girar ángulos precisos utilizando odometría diferencial, ignorando la lógica de combate.

### 🔌 Hardware Requerido (Setup Práctica)

Se utilizan dos sensores reflexivos **CNY70** sobre discos codificados (Blanco/Negro) adheridos a las ruedas.

| Función               | Pin Tiva | Periférico  | Configuración                |
| :-------------------- | :------- | :---------- | :--------------------------- |
| **Encoder Izquierdo** | `PC5`    | GPIO Port C | Interrupción (Ambos Flancos) |
| **Encoder Derecho**   | `PC6`    | GPIO Port C | Interrupción (Ambos Flancos) |

### 🧠 Lógica de Control (`src/odometry.c`)

El sistema utiliza un bucle de control bloqueante (Open Loop con feedback de posición) basado en las ecuaciones de cinemática diferencial:

1.  **Avance ($D$):** Promedio de ambas ruedas.
    $$D = \frac{Ticks_L + Ticks_R}{2} \times K_{cm/tick}$$

2.  **Giro ($\theta$):** Diferencia entre ruedas.
    $$\theta = \frac{R}{L} (Ticks_L - Ticks_R)$$

#### API Disponible (`include/odometry.h`)

- `void mover_robot(float cm);` -> Desplazamiento lineal (+/-).
- `void girar_robot(float deg);` -> Giro sobre el propio eje (+/-).

> **Nota:** Estas funciones son **bloqueantes**. No usar en la versión de combate (Strategy Loop).

---
