#define led1 2
#define led2 3
#define led3 4
#define sensor A0

void setup() {
  
pinMode (led1, OUTPUT);
pinMode (led2, OUTPUT);
pinMode (led3, OUTPUT);

Serial.begin(9600);
}

void loop() {
  int Vlrd  =  analogRead(sensor);

  Serial.println(Vlrd);
    
  if (Vlrd < 300  )
  {
    digitalWrite(led1, HIGH);
    digitalWrite(led2, HIGH);
    digitalWrite(led3, HIGH);
  } else {
           digitalWrite(led1, LOW);
           digitalWrite(led2, LOW);
           digitalWrite(led3, LOW);
         }
}
