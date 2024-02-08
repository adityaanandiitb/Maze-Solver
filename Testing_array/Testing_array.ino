void setup(){
  Serial.begin(9600);
}

void loop(){

  showSensorData();
  delay(1000);
}


void showSensorData()
{
  Serial.print("Sensor 0-  ");
  Serial.print(analogRead(0));
  Serial.print("  Sensor 1-  ");
  Serial.print(analogRead(1));
  Serial.print("  Sensor 2-  ");
  Serial.print(analogRead(2));
  Serial.print("  Sensor 3-  ");
  Serial.print(analogRead(3));
  Serial.print("  Sensor 4-  ");
  Serial.println(analogRead(4));
}