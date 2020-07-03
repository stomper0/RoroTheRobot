/* Battery Rundown Test
 * Franklyn Watson
 * 
 * Program to report on the apparent voltage of a 14.4v Roomba battery
 * Batter will be connected to analogIn via a voltage divider
 * Division will be from a max of 16v to 4.94v (due to available resistors)
 * AnalogIn must be multiplied back up to the 0-16v range then data output on Serial
 */

int analogInput;
float analogVoltage;
String outString;

void setup() {
  Serial.begin(9600);
}

void loop() {
  analogInput = analogRead(A0);                 // Get the analogIn (0-1023)
  analogVoltage = analogInput * 5.0 / 1023.0;   // Voltage = value * the ratio

  outString = millis() + " - Voltage reading: " + String(analogVoltage);

  Serial.println(outString);                    // print the string on the serial Monitor
  delay(5000);                                  // wait 5 seconds
}
