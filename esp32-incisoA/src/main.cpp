#include <Arduino.h>

#define N 10000
#define STACK_SIZE 10000
#define PRIORITY 20
// ---------------------------------------------------------
// VARIABLES GLOBALES
// ---------------------------------------------------------
int nValues[6] = {2, 4, 10, 100, 5000, 10000};
int nArraySize = 6;

int iniArray[N];
int resultArray[N];

unsigned long startTime;
unsigned long endTime;
unsigned long execOneTask, execTwoTasks;

// ---------------------------------------------------------
// ESTRUCTURA DE ARGUMENTOS PARA LAS TAREAS
// ---------------------------------------------------------
struct argsStruct {
  int arrayStart;
  int arrayEnd;
  int n;        // exponente
};

// semáforo de conteo para esperar a ambas tareas
SemaphoreHandle_t doneSem;

// ---------------------------------------------------------
// TAREA DE CÁLCULO DE POTENCIA
// ---------------------------------------------------------
void powerTask(void *parameters) {
  argsStruct myArgs = *((argsStruct*)parameters);

  int product;

  for (int i = myArgs.arrayStart; i < myArgs.arrayEnd; i++) {
    product = 1;
    for (int j = 0; j < myArgs.n; j++) {
      product = product * iniArray[i];
    }
    resultArray[i] = product;
  }
  
  xSemaphoreGive(doneSem);
  vTaskDelete(NULL);
}


// ---------------------------------------------------------
// SETUP
// ---------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("----- PROCESAMIENTO MULTI-NUCLEO ESP32 -----");

  // Crear semáforo de conteo para esperar tareas
  doneSem = xSemaphoreCreateCounting(2, 0);

  // -----------------------------------------------------
  // INICIALIZAR DATOS ALEATORIOS
  // -----------------------------------------------------
  randomSeed(analogRead(0));
  for (int i = 0; i < N; i++) {
    iniArray[i] = random(1, 10);
  }

  // -----------------------------------------------------
  // PRUEBA PARA CADA N[i]
  // -----------------------------------------------------
  for (int idx = 0; idx < nArraySize; idx++) {
    int exp = nValues[idx];

    Serial.println();
    Serial.print("=== Prueba con exponente n = ");
    Serial.println(exp);

    // -------------------- 1 TAREA (secuencial) -------------------
    argsStruct oneTask = {0, N, exp};

    startTime = micros();
    xTaskCreatePinnedToCore(
      powerTask, "oneTask", STACK_SIZE,
      (void*)&oneTask, PRIORITY,
      NULL, 0
    );
    xSemaphoreTake(doneSem, portMAX_DELAY);

    endTime = micros();
    execOneTask = endTime - startTime;
    Serial.print("Tiempo 1 tarea (us): ");
    Serial.println(execOneTask);

    // -------------------- 2 TAREAS (paralelo) --------------------
    argsStruct twoTasks1 = {0, N/2, exp};
    argsStruct twoTasks2 = {N/2, N, exp};

    startTime = micros();

    xTaskCreatePinnedToCore(
      powerTask, "twoTasks1", STACK_SIZE,
      (void*)&twoTasks1, PRIORITY,
      NULL, 0
    );

    xTaskCreatePinnedToCore(
      powerTask, "twoTasks2", STACK_SIZE,
      (void*)&twoTasks2, PRIORITY,
      NULL, 1
    );

    // esperar ambas tareas
    xSemaphoreTake(doneSem, portMAX_DELAY);
    xSemaphoreTake(doneSem, portMAX_DELAY);

    endTime = micros();

    execTwoTasks = endTime - startTime;
    Serial.print("Tiempo 2 tareas (us): ");
    Serial.println(execTwoTasks);

    // -------------------- SPEEDUP --------------------
    double speedup = (double)execOneTask / (double)execTwoTasks;
    Serial.print("Speedup = ");
    Serial.println(speedup, 4);
  }

  Serial.println("\n----- PROGRAMA TERMINADO -----");
}

void loop() { }
