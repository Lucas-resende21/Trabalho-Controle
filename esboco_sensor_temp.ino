#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 4  // pino conectado ao DATA

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup() {
  Serial.begin(115200);
  sensors.begin();
  Serial.println("Inicializando Sensor...");

  if (sensors.getDeviceCount() == 0) {
    Serial.println("Nenhum sensor encontrado!");
  } else {
    Serial.print("Sensores encontrados: ");
    Serial.println(sensors.getDeviceCount());
  }
}

void loop() {
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  if (tempC == DEVICE_DISCONNECTED_C) {
    Serial.println("Erro: Sensor desconectado!");
  } else {
    Serial.print("Temperatura: ");
    Serial.print(tempC);
    Serial.println(" °C");
  }

  delay(1000);
}
