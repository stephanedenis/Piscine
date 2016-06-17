
/*
 * Pool monitor and remote control
 *
 * By Stephane Denis
 * Under MIT License
 */

#define LDRPIN      19  // Light sensor
#define ORPPIN      PC1 // Pool Free Chlorine = Oxidation/Reduction Potential (ORP)
#define PHPIN       PC0 // Pool Ph
#define DS18B20PIN  16 // Pool temperature
#define DHTPIN      17 // Air temperature and relative humidity


#include <OneWire.h>
OneWire  ds(DS18B20PIN);  // Pool temperature sensor (a 4.7K resistor is necessary)

#include "DHT.h"
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
// Nokia 5110 Display - Software SPI (slower updates, more flexible pin options):
// pin 7 - Serial clock out (SCLK)
// pin 6 - Serial data out (DIN)
// pin 5 - Data/Command select (D/C)
// pin 4 - LCD chip select (CS)
// pin 3 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(7, 6, 5, 4, 3);

byte i;
byte present = 0;
byte type_s;
byte data[12];
byte addr[8];

void setup() {
  Serial.begin(9600);

  display.begin();
  display.setContrast(50);
  display.setTextSize(1);
  display.setTextColor(BLACK);

  // LED Backlight
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Environment
  dht.begin();


  // Pool thermometer
  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
  }

  byte addr[8];
  Serial.print("ROM =");
  for ( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
  }
  Serial.println();

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
  }
}

void loop() {

  float light = analogRead(LDRPIN) / 1024.0;
  float airHumidity = dht.readHumidity() / 100.0;
  float airCelsius = dht.readTemperature();
  float airFarenheit = dht.readTemperature(true);
  float poolCelsius = getPoolTemperature();
  float poolFarenheit = poolCelsius * 1.8 + 32.0;
  float poolPh = 7 - (2.5 - analogRead(PHPIN) / 200.0) / (0.257179 + 0.000941468 * poolCelsius);
  float poolORP = (2.5 - analogRead(ORPPIN) / 200.0) / 1.037;


  Serial.print("{");
  Serial.print("\"light\":");
  Serial.print(light);
  Serial.print(",\"airHumidity\":");
  Serial.print(airHumidity);
  Serial.print(",\"airCelsius\":");
  Serial.print(airCelsius);
  Serial.print(",\"airFarenheit\":");
  Serial.print(airFarenheit);
  Serial.print(",\"poolCelsius\":");
  Serial.print(poolCelsius);
  Serial.print(",\"poolFarenheit\":");
  Serial.print(poolFarenheit);
  Serial.print(",\"poolPh\":");
  Serial.print(poolPh);
  Serial.print(",\"poolORP\":");
  Serial.print(poolORP);
  Serial.println("}");

  display.clearDisplay();
  display.setCursor(0, 0);

  // Check if any reads failed and exit early (to try again).
  if (isnan(airHumidity) || isnan(airCelsius) || isnan(airFarenheit)) {
    Serial.println("Failed to read from DHT sensor!");
  }
  else {
    display.println("Environnement");
    display.println();

    display.print("T:");
    display.print(int(airCelsius));
    display.print("C, H:");
    display.print(int(airHumidity * 100));
    display.println("%");
    display.println();
  }
  display.print("Lumiere:");
  display.println(light * 100);
  display.display();

  delay(2000);
  display.clearDisplay();
  display.setCursor(0, 0);

  display.println("Eau");
  display.println();

  display.print("T:");
  display.print(int(poolCelsius));
  display.print("C ");
  display.print(int(poolFarenheit));
  display.println("F ");
  display.println();

  display.print("Ph:");
  display.println(poolPh);
  display.print("ORP:");
  display.print(poolORP);
  display.println();

  display.display();
  delay(2000);
}


float getPoolTemperature() {
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad
  /*
    Serial.print("  Data = ");
    Serial.print(present, HEX);
    Serial.print(" ");
    */
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    //Serial.print(data[i], HEX);
    //Serial.print(" ");
  }
  /*
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();
  */
  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }

  return (float)raw / 16.0;
}

