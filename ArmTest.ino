#include <Servo.h>

Servo s1, s2, s2m, s3, s4, s5;

void setup() {
  Serial.begin(115200);

  s1.attach(2);  
  s2.attach(3);  
  s2m.attach(5); 
  s3.attach(6);  
  s4.attach(9);  
  s5.attach(10); 
}

void loop() {
  static String buffer = "";

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      processLine(buffer);
      buffer = "";
    } else {
      buffer += c;
    }
  }
}

void processLine(String line) {
  line.trim();

  int a1, a2, a3, a4, a5;

  if (sscanf(line.c_str(), "%d,%d,%d,%d,%d", &a1, &a2, &a3, &a4, &a5) == 5) {

    s1.write(a1);
    s2.write(a2);
    s2m.write(180 - a2);
    s3.write(a3);
    s4.write(a4);
    s5.write(a5);

    Serial.println("OK");
  }
}
