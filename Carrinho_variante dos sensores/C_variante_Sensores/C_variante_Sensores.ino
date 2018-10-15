/*
   Curso de Arduino e AVR WR Kits Channel 
   Aula 72 - Robô com Motor Shield (Parte 2)
   Codigo modificado (2 sensores, + led's de indicação de tomada de decisão);
*/
// --- Bibliotecas Auxiliares ---
#include <AFMotor.h>         //Inclui biblioteca AF Motor
#include <Servo.h>           //Inclui biblioteca para controle de Servos
 
// --- Seleção dos Motores ---
AF_DCMotor motor1(1); //Seleção do Motor 1
AF_DCMotor motor2(2); //Seleção do Motor 1
AF_DCMotor motor3(3); //Seleção do Motor 1
AF_DCMotor motor4(4); //Seleção do Motor 1
 
// --- Mapeamento de Hardware ---
#define   serv        10                 //controle do Servo 1
#define   trig        A2                 //Saída para o pino de trigger do sensor
#define   echo        A3                 //Entrada para o pino de echo do sensor

                  // --- definição dos sensores --- 
                    // ( As saidas do Shied, todas as 6 (de A0 a A5) podem ser usadas tanto como Analogicas ou digitais, para definição, 
                    // precisamos nomealas pela letra A seguida do digito desejado.
                    
                     #define pinSensor1 A0  // Definição do sensor na porta Analogica 0 (sensor frente)
                     #define pinSensor2 A1  // Definição do sensor na porta Analogica 1 (sensor tras)
                  
                  // --- definição do led --- 
                  
                     #define Red A2    // Definição do led vermelho (Analogico 2, indica uso da função de tomada de decisão)
                     #define Green A3  // Definição do led Verde (Analogica 3, indica pleno funcionamento da função loop, = Obistaculos não localizados)
                     
// --- Protótipo das Funções Auxiliares ---

float measureDistance();                //Função para medir, calcular e retornar a distância em cm
void trigPulse();                       //Função que gera o pulso de trigger de 10µs
void decision();                        //Função para tomada de decisão. Qual melhor caminho?
void robot_forward(unsigned char v);    //Função para movimentar robô para frente
void robot_backward(unsigned char v);   //Função para movimentar robô para trás
void robot_left(unsigned char v);       //Função para movimentar robô para esquerda
void robot_right(unsigned char v);      //Função para movimentar robô para direita
void robot_stop(unsigned char v);       //Função para parar o robô
void Sensores();                        //Função dos sensores

// --- Objetos ---
Servo servo1;                           //Criação do objeto -> para contorle do servo motor
 
// --- Variáveis Globais ---
unsigned char velocidade = 0x00;       //Armazena a velocidade dos motores (8 bits)
float dist_cm;                         //Armazena a distância em centímetros entre o robô e o obstáculo
float dist_right;                      //Armazena a distância em centímetros da direita
float dist_left;                       //Armazena a distância em centímetros da esquerda
          
// --- Configurações Iniciais ---

void setup()
{
          // --- Abertura da serial para monitoramento (sensores frontal e traseiro unicamente) 
              Serial.begin(9600);   // <- Na velocidade de 9600 

          // --- setando os Sensores 
            // sensores digitais so aceitam 2 estados. 
            // 1 = INPUT = ENTRADA
            // 2 = OUTPUT = SAIDA
              
              pinMode (pinSensor1, INPUT);  // Setando a porta A0 como Entrada; 
              pinMode (pinSensor2, INPUT);  // Setando a porta A1 como Entrada;
          
          // --- Leds
              pinMode (Red, OUTPUT);        // Setando a porta A2 como Saida; 
              pinMode (Green, OUTPUT);      // Setando a porta A3 como Saida;
              
                
  //A biblioteca configura as entradas e saídas pertinentes ao Motor Shield...
    
 pinMode(trig, OUTPUT);                       //Saída para o pulso de trigger
 pinMode(serv, OUTPUT);                       //Saída para o servo motor
 pinMode(echo, INPUT);                        //Entrada para o pulso de echo
  
 servo1.attach(serv);                         //Objeto servo1 no pino de saída do servo
 digitalWrite(trig, LOW);                     //Pino de trigger inicia em low
 servo1.write(80);                            //Centraliza servo
 delay(500);                                  //Aguarda meio segundo antes de iniciar
  velocidade = 0xFF; //Inicia velocidade no valor máximo
} //end setup
 
// --- Loop Infinito ---
void loop()
{
  digitalWrite(Red, LOW);  // Desligando o Led vermelho assim que o programa iniciar;
  digitalWrite(Green, HIGH);  // Ligando o Led Verde assim que o programa iniciar;
  
     robot_forward(velocidade);   // Movendo o carrinho para frente (velocidade e configurada por uma variavel)
     delay(40);    // <- Mudado 13/09  (controlando por quantos segundos o carrinho vai se mover antes de começar a localizar obstaculos)
     dist_cm = measureDistance(); // setando na variavel dist_cm os valores obtidos do sensor ultrasonico.
                if(dist_cm != 0){
              Serial.println("armazenando na dist_cm o retorno da função measureDistance();");
              Serial.println(dist_cm,DEC);
              Serial.println(".");
            }else{
              Serial.println("valor de dist_cm da função measureDistance() não encontrado");
              Serial.println(dist_cm,DEC);
              Serial.println(".");
            }
     if(dist_cm < 30) // caso algum valor seja obtido e a distância seja menor que 20 cm... faça;
     {
         decision(); // chamando a função de decisão;
     }
} //end loop
 
// --- Desenvolvimento das Funções Auxiliares ---
 
float measureDistance()                       //Função que retorna a distância em centímetros
{
  float pulse;                                //Armazena o valor de tempo em µs que o pino echo fica em nível alto
  trigPulse();                                //Envia pulso de 10µs para o pino de trigger do sensor
  pulse = pulseIn(echo, HIGH);                //Mede o tempo em que echo fica em nível alto e armazena na variável pulse
  /*
    >>> Cálculo da Conversão de µs para cm:
   Velocidade do som = 340 m/s = 34000 cm/s
   1 segundo = 1000000 micro segundos
    
      1000000 µs - 34000 cm/s
            X µs - 1 cm
             
                  1E6
            X = ------- = 29.41
                 34000
                  
    Para compensar o ECHO (ida e volta do ultrassom) multiplica-se por 2
     
    X' = 29.41 x 2 = 58.82
 */
   
  return (pulse/58.82);                      //Calcula distância em centímetros e retorna o valor
} //end measureDistante
 
void trigPulse()                             //Função para gerar o pulso de trigger para o sensor HC-SR04
{
   digitalWrite(trig,HIGH);                  //Saída de trigger em nível alto
   delayMicroseconds(10);                    //Por 10µs ...
   digitalWrite(trig,LOW);                   //Saída de trigger volta a nível baixo
} //end trigPulse
 
void decision()                              //Compara as distâncias e decide qual melhor caminho a seguir
{
  digitalWrite(Red, HIGH);                   // Ligando o Led Vermelho (indicação que o carrinho chamou a função de decisão)
  digitalWrite(Green, LOW);                  // Desligando o Led verde.
  
   robot_stop(velocidade);                   //Para o robô
   delay(500);                               //Aguarda 500ms
   servo1.write(0);                          //Move sensor para direita através do servo
   delay(500);                               //Aguarda 500ms
   dist_right = measureDistance();           //Mede distância e armazena em dist_right
   delay(2000);                              //Aguarda 2000ms
   servo1.write(150);                        //Move sensor para esquerda através do servo
   delay(500);                               //Aguarda 500ms
   dist_left = measureDistance();            //Mede distância e armazena em dis_left
   delay(2000);                               //Aguarda 2000ms
   servo1.write(70);                         //Centraliza servo. - = direita, + = esquerda  
   delay(500);
   if(dist_right > dist_left)                //Distância da direita maior que da esquerda?
   {                                         //Sim...
      robot_backward(velocidade);            //Move o robô para trás
      delay(200);                            //Por 200ms
      robot_right(velocidade);               //Move o robô para direita
      delay(200);                            //Por 2000ms  <- Mudado 13/09
      robot_forward(velocidade);             //Move o robô para frente
   } //end if
   
    if(dist_left > dist_right)               //Distância da esquerda maior que da direita?
   {                                         //Sim
    robot_backward(velocidade);              //Move o robô para trás
      delay(200);                            //Por 200ms
      robot_left(velocidade);                //Move o robô para esquerda
      delay(200);                            //Por 2000ms      <- Mudado 13/09
      robot_forward(velocidade);              //Move o robô para frente
   } //end if
    digitalWrite(Green, HIGH);                // Acendendo o Led verde = indicação de fim da tomada de decisão.
} //end decision
  
void robot_forward(unsigned char v)
{
     motor1.setSpeed(v);
     motor1.run(FORWARD);
     motor2.setSpeed(v);
     motor2.run(FORWARD);
     motor3.setSpeed(v);
     motor3.run(FORWARD);
     motor4.setSpeed(v);
     motor4.run(FORWARD);
} //end robot forward
 
void robot_backward(unsigned char v)
{
     motor1.setSpeed(v);
     motor1.run(BACKWARD);
     motor2.setSpeed(v);
     motor2.run(BACKWARD);
     motor3.setSpeed(v);
     motor3.run(BACKWARD);
     motor4.setSpeed(v);
     motor4.run(BACKWARD);
} //end robot backward
 
void robot_left(unsigned char v)
{
     motor1.setSpeed(v);
     motor1.run(FORWARD);
     motor2.setSpeed(v);
     motor2.run(FORWARD);
     motor3.setSpeed(v);
     motor3.run(BACKWARD);
     motor4.setSpeed(v);
     motor4.run(BACKWARD);
} //end robot left
 
void robot_right(unsigned char v)
{
     motor1.setSpeed(v);
     motor1.run(BACKWARD);
     motor2.setSpeed(v);
     motor2.run(BACKWARD);
     motor3.setSpeed(v);
     motor3.run(FORWARD);
     motor4.setSpeed(v);
     motor4.run(FORWARD);
} //end robot right
 
void robot_stop(unsigned char v)
{
     motor1.setSpeed(v);
     motor1.run(RELEASE);
     motor2.setSpeed(v);
     motor2.run(RELEASE);
     motor3.setSpeed(v);
     motor3.run(RELEASE);
     motor4.setSpeed(v);
     motor4.run(RELEASE);
} //end robot stop

// --- Conotrole dos sensores 

void Sensores(){
  
     bool estadoSensor1 = digitalRead(pinSensor1);             //criando o Bool que indicara se estamos recebendo reflexo ou não;
     bool estadoSensor2 = digitalRead(pinSensor2);             //criando o Bool que indicara se estamos recebendo reflexo ou não;

     if(estadoSensor1 or estadoSensor2)                        // testando se, der reflexo no Sensor 1 ou no Sensor 2
        {
          Serial.println("Linha detectada!");                  // Printando na serial.
             if(estadoSensor1)                                 // caso a linha detectada seja na no sensor 1
                {
                Serial.println("Linha na frente.");            // Printa
                robot_stop(velocidade);                        // Para o robo.
              }
          if(estadoSensor2)                                    // caso a linha detectada seja na no sensor 1
              {
                Serial.println("Linha Atras.");                // Printa
                robot_stop(velocidade);
              }       
         } else {                                              // Caso nada seja detectado pelos sensores  
                 Serial.println("Nenhuma Linha detectada.");   // Printa
              }
}
