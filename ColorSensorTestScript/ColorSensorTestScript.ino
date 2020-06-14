void setup() {
  Serial.begin(9600);

  if (leftColourSensor.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
  }
}

void loop() {
  uint16_t r, g, b, c, colorTemp, lux;

  leftColourSensor.getRawData(&r, &g, &b, &c);
  // colorTemp = leftColourSensor.calculateColorTemperature(r, g, b);
  colorTemp = leftColourSensor.calculateColorTemperature_dn40(r, g, b, c);
  lux = leftColourSensor.calculateLux(r, g, b);

  Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");
}
