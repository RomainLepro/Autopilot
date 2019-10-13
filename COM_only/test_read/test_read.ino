


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
Serial.println(float(analogRead(A3))*180.0/1024.0);
Serial.println(float(analogRead(A2))*180.0/1024.0);
Serial.println("");
}
