#include <LiquidCrystal.h>

const int pingPin = 7;

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

long calibrationPoint;

long microsecondsToMillimeters(long microseconds)
{
// The speed of sound is 340 m/s or 29 microseconds per centimeter,
// and then we multiply with 10 to get in in millimeters.
// The ping travels out and back, so to find the distance of the
// object we take half of the distance travelled.

// 340 m/s
// 34000 cm/s
// 340000 mm/s
// 340 mm/ms
// 0.34 mm/micros

// 1/0.34 mm/micros = 2.9411 micros/mm
return microseconds / 2.9411 / 2;
}

void setup() {
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);

  lcd.print("G'day my friend!");
  
  Serial.begin(9600);

  long duration;

  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  calibrationPoint = microsecondsToMillimeters(duration);
  
}

void loop() {
  
  long duration, inches, cm, mm, thicknest;

  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  // convert the time into a distance
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);
  mm = microsecondsToMillimeters(duration);

  thicknest = calibrationPoint - mm;

  
//  Serial.print(inches);
//  Serial.print("in, ");
//  Serial.print(cm);
//  Serial.print("cm");
//  Serial.println();

  delay(300);

  lcd.setCursor(0, 1);
  
  String thicknestPrint = String(thicknest);
  String smm = String(mm);
  
  String lcdSecondLine = "Thicknest: " +thicknestPrint + "mm ";

  lcd.print(lcdSecondLine);
  
}

long microsecondsToInches(long microseconds) {
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
  
}


 
