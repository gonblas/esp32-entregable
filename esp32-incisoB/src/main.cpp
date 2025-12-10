/*----------------------------------------------------------------------------------------------------------------------------------------------*/
// Definiciones para habilitar/deshabilitar distintas porciones de codigo --> 0L desactivo, 1L activa                                         // |
#define SENSORES_ENABLE 1L // |
#define MQTT_ENABLE 0L
#define WIFI_ENABLE 0L   // |
#define LORA_ENABLE 1L   // |                                                                                                         // |
#define PRUEBA_ENABLE 0L // simulacion de sensores mediante generacion de numeros aleatorio en el rango adecuado usando funcion "random"   |
#define RELE_ENABLE 0L   //
#define PID_CONTROLER 1L // HABILITA CONTROLADOR PID
/*----------------------------------------------------------------------------------------------------------------------------------------------*/

#include <Arduino.h>

#if PID_CONTROLER
#include <PIDController.hpp>
#endif

#if WIFI_ENABLE
#include <WiFi.h>
#endif

#if MQTT_ENABLE
#include <MQTTPubSubClient.h>
// BROKER EMQX
#define AIO_SERVER "163.10.3.73"
#define AIO_SERVERPORT 1883 // use 8883 for SSL
#define AIO_USERNAME "embebidos"
#define AIO_KEY "Digitales1"
// DEFINE EL CLIENTE MQTT
WiFiClient client;
MQTTPubSubClient mqtt;
#endif

// https://www.hackster.io/HARGOVIND/nodemcu-based-iot-project-connecting-yl-69-yl-38-moisture-7cf84a calibracion del sensor

#if SENSORES_ENABLE
String iluminacion;
String iluminacion2;
const int PIN_LDR = 32;
const int PIN_LDR2 = 33;
const long A = 1000; // Resistencia en oscuridad en KΩ
const int B = 15;    // Resistencia a la luz (10 Lux) en KΩ
const int Rc = 10;   // Resistencia calibracion en KΩ

float readLDR()
{
  float mean = 0;
  for (auto i = 0; i < 100; i++)
  {
    auto V = (float)analogRead(PIN_LDR);
    mean += (V * A * 10) / (B * Rc * (1024 - V));
  }
  mean /= 100;
  return mean;
}

float readLDR2()
{
  float mean = 0;
  for (auto i = 0; i < 100; i++)
  {
    auto V = (float)analogRead(PIN_LDR2);
    mean += (V * A * 10) / (B * Rc * (1024 - V));
  }
  mean /= 100;
  return mean;
}
#endif

#if PID_CONTROLER

const int PIN_LED = 12;
const int PIN_LED2 = 14;

PID::PIDParameters<double> parameters(0.4, 6.0, 0.0001); // parámetros del controlador PID
PID::PIDController<double> pidController(parameters);
#endif

#if WIFI_ENABLE
// Reemplazar con las credenciales de la red a usar
const char *ssid = "Wokwi-GUEST";
const char *password = "";

#endif

// Funcion que va a ejecutarse cuando se active la interrupcion externa al pulsar el boton

#if WIFI_ENABLE
void connectWiFi()
{
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP()); // Después de que finaliza la conexión se imprime la IP local del ESP32 en la red, ya que
                                  // se necesita para conectar al servidor usando un navegador web.
}
#endif

#if MQTT_ENABLE

void connectMQTT()
{
  Serial.print("connecting to host...");
  while (!client.connect("163.10.3.73", 1883))
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println(" connected!");

  // initialize mqtt client
  mqtt.begin(client);

  Serial.print("connecting to mqtt broker...");
  while (!mqtt.connect("esp32", "embebidos", "Digitales1"))
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println(" connected!");

  // subscribe callback which is called when every packet has come
  mqtt.subscribe([](const String &topic, const String &payload, const size_t size)
                 { Serial.println("mqtt received: " + topic + " - " + payload); });

  // subscribe topic and callback which is called when /hello has come
}

#endif

#if SENSORES_ENABLE
void SensorTask(void *pvParameters)
{
  (void)pvParameters;

  for (;;)
  {
    float ilum2 = readLDR2();
    ; // sensor de luz exterior control on-off
    iluminacion2 = String(ilum2);
    Serial.println("Iluminacion exterior: ");
    Serial.println(ilum2);
    if (ilum2 < 200)
      digitalWrite(PIN_LED2, HIGH);
    else
      digitalWrite(PIN_LED2, LOW);

    auto ilum = readLDR(); // sensor de luz intrerior control PID
    Serial.println("Iluminacion interior: ");
    Serial.println(ilum);
    iluminacion = String(ilum);
    Serial.println(ilum);
    pidController.Input = ilum;
    pidController.Update();
    analogWrite(PIN_LED, (int)pidController.Output);
  }
}
#endif

#if MQTT_ENABLE
void MQTTTask(void *pvParameters)
{
  (void)pvParameters;

  for (;;)
  {
    mqtt.update(); // should be called
    // publish message
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 1000)
    {
      prev_ms = millis();

      mqtt.publish("/grupo1/iluminacion", iluminacion);
      delay(500);
      mqtt.publish("/grupo1/iluminacion2", iluminacion2);
      delay(500);
    }
  }
}
#endif

void setup()
{
  Serial.begin(115200);
  delay(1000); // Esperar a que el puerto serial se inicialice
  Serial.println("Inicializando");

  pinMode(PIN_LDR, INPUT);
  pinMode(PIN_LDR2, INPUT);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);

  pidController.Input = analogRead(PIN_LDR);
  pidController.Setpoint = 340;
  pidController.TurnOn();
}

void loop()
{
  delay(1000); // Delay para evitar saturar el puerto serial
  float ilum = readLDR();
  float ilum2 = readLDR2();

  Serial.print("Luz interior: ");
  Serial.println(ilum);

  Serial.print("Luz exterior: ");
  Serial.println(ilum2);

  // LED exterior ON/OFF
  if (ilum2 < 200)
    digitalWrite(PIN_LED2, HIGH);
  else
    digitalWrite(PIN_LED2, LOW);

  // PID interior
  pidController.Input = ilum;
  pidController.Update();
  analogWrite(PIN_LED, (int)pidController.Output);

  delay(300);
}
