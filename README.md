# Arquitectura Avanzada de Procesadores - Ejercicio entregable ESP32

**1)** Usando el compilador arduino ide para el kit esp32, compile y programe tareas que se ejecuten en los distintos cores.

**a.** Calcule el speedup entre el tiempo de ejecución de un algoritmo que calcula la potencia de un número, comparando la ejecución en una tarea
y dos tareas repartidas entre ambos núcleos (Siga los pasos indicados en la presentación de clase).

**b.** Realice una aplicación que lea entradas analógicas (ejemplo sensor de iluminación) y aplique un algoritmo de control (control PID), para luego
enviar los valores medidos por wifi. distribuya las tareas para que la ejecución sea más eficiente (tenga en cuenta que uno de los núcleos se
encarga de la comunicación wifi). 

> Para las pruebas podrá solicitar un kit esp32 durante el horario de clases o en horarios a coordinar con el docente.