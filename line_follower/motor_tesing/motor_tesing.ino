int aenbl = 3;
int aphase =2;
int benbl = 5;
int bphase = 4;


void setup() {
  // put your setup code here, to run once:
  // pinMode(aenbl, OUTPUT);
  // pinMode(aphase, OUTPUT);
  // pinMode(benbl, OUTPUT);
  // pinMode(bphase, OUTPUT);
  
  analogWrite(aenbl, 250);
  analogWrite(aphase, 0);
  analogWrite(benbl, 0);
  analogWrite(bphase, 250);
  Serial.begin(9600);

}

char ch;
int speed = 100;
void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    ch = Serial.read();
    if (ch == 'a') {
      speed += 10;
    } else if (ch == 's') {
      speed -= 10;
    }
  }

  Serial.println(speed);
    
  analogWrite(aenbl, 0);
  analogWrite(aphase, speed);
  analogWrite(benbl, speed);
  analogWrite(bphase, 0);


}
