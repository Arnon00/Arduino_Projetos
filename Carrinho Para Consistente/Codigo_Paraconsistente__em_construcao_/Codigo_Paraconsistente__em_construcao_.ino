// --- Bibliotecas Auxiliares ---
#include <Ultrasonic.h>      // Incluindo a biblioteca do Ultrassonico;
#include <AFMotor.h>         //Inclui biblioteca AF Motor;
 
// --- Seleção dos Motores ---
AF_DCMotor motor1(1); //Seleção do Motor 1
AF_DCMotor motor2(2); //Seleção do Motor 1
AF_DCMotor motor3(3); //Seleção do Motor 1
AF_DCMotor motor4(4); //Seleção do Motor 1
 
//------Define os pinos utilizados pelos sensores------

#define   trige        A2                 //Saída para o pino de trigger do sensor Esquerda   - no codigo original = #define pino_trigger 2
#define   echoe        A3                 //Entrada para o pino de echo do sensor Esquerda    - no codigo original = #define pino_echo 3
#define   trigd        A4                 //Saída para o pino de trigger do sensor Direita    - no codigo original = #define pino_trigger2 7
#define   echod        A5                 //Entrada para o pino de echo do sensor Direita     - no codigo original = #define pino_echo2 8

//------Iicializando os sensores------
// --- definição dos sensores --- 
                    // ( As saidas do Shied, todas as 6 (de A0 a A5) podem ser usadas tanto como Analogicas ou digitais, para definição, 
                    // precisamos nomealas pela letra A seguida do digito desejado.
                    
                     #define pinSensor1 A0  // Definição do sensor na porta Analogica 0 (sensor direita)
                     #define pinSensor2 A1  // Definição do sensor na porta Analogica 1 (sensor esquerda)
                  
Ultrasonic sensor1(trigd, echod);            // Iniciando - sensor 1 - com os valores do trig e echo do sensor a direita;
Ultrasonic sensor2(trige, echoe);            // Iniciando - sensor 2 - com os valores do trig e echo do sensor a esquerda;

//------Variaveis de controle------

float microsec_esq, e2, microsec_dir, d1;
unsigned char velocidade = 150;       //Armazena a velocidade dos motores (8 bits)
float dist1, dist2;                           // Valores de comparação das distancias obtidas;
float vscc = 0.5;                             // Valor Superior de Controle de Certeza V;
float vicc = -0.5;                            // Valor Inferior de Controle de Certeza F;
float vscct = 0.5;                            // Valor Superior de Controle de Contradicao T;
float vicct = -0.5;                           // Valor Inferior de Controle de Contradicao;

// ------Funcoes---------

/*
 * Funções de controle.
*/

void robot_forward(unsigned char v) // Função para mover -> Frente 
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
 
void robot_backward(unsigned char v) // Função para mover -> Ré
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
 
void robot_left(unsigned char v) // Função para mover ->  Esquerda 
{
     motor2.setSpeed(v);
     motor2.run(FORWARD);
     motor3.setSpeed(v);
     motor3.run(FORWARD);
     motor1.setSpeed(v);
     motor1.run(BACKWARD);
     motor4.setSpeed(v);
     motor4.run(BACKWARD);
} //end robot left

void virada_leve_esquerda (unsigned char v)// Função para mover -> Virada esquerda enquanto movimento 
{
     motor2.setSpeed(v);
     motor2.run(BACKWARD);
     motor3.setSpeed(v);
     motor3.run(BACKWARD);
     motor1.setSpeed(200);
     motor1.run(FORWARD);
     motor4.setSpeed(200);
     motor4.run(FORWARD);
}

void robot_right(unsigned char v) // Função para mover -> Direita 
{
     motor2.setSpeed(v);
     motor2.run(BACKWARD);
     motor3.setSpeed(v);
     motor3.run(BACKWARD);
     motor1.setSpeed(v);
     motor1.run(FORWARD);
     motor4.setSpeed(v);
     motor4.run(FORWARD);
} //end robot right

void virada_leve_direita (unsigned char v) // Função para mover -> direita enquanto movimento
{
      motor2.setSpeed(200);
      motor2.run(FORWARD);
      motor3.setSpeed(200);
      motor3.run(FORWARD);
      motor1.setSpeed(v);
      motor1.run(BACKWARD);
      motor4.setSpeed(v);
      motor4.run(BACKWARD);
}
 
void robot_stop(unsigned char v) // Função para mover -> parar
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

/*
 * Função de analise dos sensores de linha.
*/
void Sensores(){
  
     bool estadoSensor1 = digitalRead(pinSensor1);             //criando o Bool que indicara se estamos recebendo reflexo ou não;
     bool estadoSensor2 = digitalRead(pinSensor2);             //criando o Bool que indicara se estamos recebendo reflexo ou não;

     if(estadoSensor1 or estadoSensor2)                        // testando se, der reflexo no Sensor 1 ou no Sensor 2
        {
          Serial.println("Linha detectada!");                  // Printando na serial.
             if(estadoSensor1)                                 // caso a linha detectada seja na no sensor 1
                {
                Serial.println("Linha a Direita.");            // Printa
                robot_stop(velocidade);                        // Para o robo.
              }
          if(estadoSensor2)                                    // caso a linha detectada seja na no sensor 1
              {
                Serial.println("Linha a Esquerda.");                // Printa
                robot_stop(velocidade);
              }       
         } else {                                              // Caso nada seja detectado pelos sensores  
                 // Serial.println("Nenhuma Linha detectada.");   // Printa
              }
}

/*
 * Função de analise dos ultrasonicos.
*/
float distancia(Ultrasonic sensor)            //Retorna a distancia medida pelos sensores em um limite de 5cm a 200cm;
{
  float cmMsec;                                                  // Valor para conversão das distancias;
  long microsec = sensor.timing();                               // <- armazendo em um LONG (maior valor de float) o valor retornado diretamente do sensor;
  cmMsec = sensor.convert(microsec, Ultrasonic::CM);             // Convertendo o valor de long para float, (Valor em centimetros);
  if (cmMsec > 200) {                                            // Se o valor de cmMsec for maior que 200, o mesmo recebe o valor de 200; 
    cmMsec = 200;
  } else {                                                       // Se o valor de cmMsec for menor que 5, o mesmo recebe o valor de 5; 
    if (cmMsec < 5) {
      cmMsec = 5;
    }
  }
  return cmMsec;                                                 // Retornando cmMsec <- float. com o valor de 5 ou 200 para quem o chamou;
}
/*
 * Função de analise da logica paraconsistente.
 
 * Aqui, sera usado os valores alterados pelo MAP, para alcançar uma (%) de chance de colizão - do 0% a 100%, passados como parametro na chamada da função.
 *
 *
 * Ref: http://playground.arduino.cc/referencia/else
*/
int paraAnalisador (float certeza, float incerteza) 
    {          
        certeza = certeza / 100;                                                            // Divide o valor por 100 e armazeno na float (Lembrando, o ultra retorna um valor Long);
        incerteza = incerteza / 100;                                                        // Divide o valor por 100 e armazeno na float (Lembrando, o ultra retorna um valor Long);
          float gC = certeza - incerteza;                                                   // cauculo da crteza - incerteza -> armazenado em gC;
          float gI = ((certeza + incerteza) - 1);                                           // cauculo da crteza + incerteza - 1 -> armazenado em gI;
          int sR = 0;                                                                       
          float modgC, modgI;
            if (gC < 0) modgC = gC * (-1);                   // <----  (?)
            else modgC = gC;                                 // <----  (?)
            if (gI < 0) modgI = gI * (-1);                   // <----  (?)
            else modgI = gI;
          
            //Determinando os Estados Extremos                            
            /* Gerando os valores de S1 usando os controles de incerteza(?)
            */
            if (gC >= vscc)sR = 1;                                // Verdadeiro - Vai Bater
            else if (gC <= vicc)sR = 2;                           // Falso - Não Vai Bater
            else if (gI >= vscct)sR = 3;                          // Inconsistente - Virar para Direita
            else if (gI <= vicct)sR = 4;                          // Para completo - Virar para Esquerda
          
            //Determinando os Estados Não Extremos
            if ((gC >= 0 and gC < vscc) and (gI >= 0 and gI < vscct)) {
              if (gC >= gI) sR = 5; //Quase Verdadeiro Tendendo ao Inconsistente qvt
              else sR = 6; //Inconsistente Tendendo ao Verdadeiro tv
            } else {
              if ((gC >= 0 and gC < vscc) and (vicct<=gI and gI < 0)) {
                if (gC >= modgI) sR = 7; // Quase Verdadeiro Tendendo ao Indeterminado
                else sR = 8; //Indeterminado Tendendo ao Verdadeiro
              } else {
                if ((vscc < gC and gC <= 0) and (vicct < gI and gI <= 0)) {
                  if (modgC >= modgI) sR = 9; // quase Falso Tendendo ao Indeterminado
                  else sR = 10; // Indeterminado Tendendo ao Falso
                } else {
                  if ((vicc <= gC and gC <= 0) and (gI >= 0 and gI < vscct)) {
                    if (modgC >= gI) sR = 11; //Quase Falso tendendo ao Inconsistente
                    else sR = 12; //Inconsistente tendendo ao Falso
                  }
                }
              }
            }
            return sR;
     }

void parada_emergencia(float d1, float e2){
 
    robot_stop(velocidade);
    delay(100);
    Serial.print("Direita: "); Serial.println(d1,DEC);
    Serial.print("Esquerda: "); Serial.println(e2,DEC);

    robot_backward(velocidade); delay(200);
}
void parada_emergencia(){
    robot_stop(velocidade);
    Serial.println("parado"); 
}

void decisao(int s1){                                  // Quando ouver Obstaculos a menos de 20 cemtimetros.
  
          if (s1 == 1){
            Serial.println("Vai Bater"); 
            parada_emergencia();
          }
         if (s1 == 3){
                Serial.println("Virar para Direita");
                velocidade = 100;
                robot_backward(velocidade);            //Move o robô para trás
                delay(200);                            //Por 200ms
                robot_right(velocidade);               //Move o robô para direita
                delay(600);                            //Por 600ms 
                robot_forward(velocidade);             //Move o robô para frente
                delay (600);
                robot_left(velocidade);                //Move o robô para esquerda
                delay (600); 
                robot_forward(velocidade);             //Move o robô para frente
              }
          if (s1 == 4){ 
                Serial.println("Virar para Esquerda");
                robot_backward(velocidade);                   //Move o robô para trás
                delay(200);                                   //Por 200ms
                robot_left(velocidade);                       //Move o robô para esquerda
                delay(600);                                   //Por 2000ms 
                robot_forward(velocidade);                    //Move o robô para frente
                delay (600);
                robot_right(velocidade);                      //Move o robô para direita
                delay(600); 
                robot_forward(velocidade);                    //Move o robô para frente
              }
velocidade = 100; robot_forward(velocidade);
}
void decisao(int s1, unsigned char v){                        // Quando ouver Obstaculos a menos de 50 cemtimetros.
  
          if (s1 == 1){ robot_forward(v);}  
          if (s1 == 3){
            Serial.println("Virando para Direita");
             virada_leve_direita(velocidade);               //Move o robô para direita
              delay(200);                                   //Por 2000ms  <- Mudado 13/09
             virada_leve_esquerda(velocidade);              //Move o robô para esquerda
              delay(200);                                   //Por 2000ms      <- Mudado 13/09
              robot_forward(velocidade);}                   //Move o robô para frente
          if (s1 == 4){ 
             Serial.println("Virando para Esquerda");
              virada_leve_esquerda(velocidade);             //Move o robô para esquerda
                  delay(200);                               //Por 2000ms      <- Mudado 13/09
              virada_leve_direita(velocidade);              //Move o robô para direita
                  delay(200);                               //Por 2000ms  <- Mudado 13/09
              robot_forward(velocidade);}                   //Move o robô para frente
                        
velocidade = 100; robot_forward(velocidade);
}
//------Inicio------
void setup() {
  Serial.begin(9600);                                  // Iniciando a serial;
              
              pinMode (pinSensor1, INPUT);  // Setando a porta A0 como Entrada; 
              pinMode (pinSensor2, INPUT);  // Setando a porta A1 como Entrada;
 }

/*
 * Map(); como o nome sugere, mapeia e altera os limites dos valores na sua primeira referencia, nesta caso, os valores obtidos do ultrassonicos.
 * Ex: valor achado foi 30.0000000, pelo controle do IF na função distancia(); teremos o retorno de um valor float de 30.0000

 * O map alterara o limite, transformando o 30.0000 em 30 <- inteiro. Porem, ele so trabalha da faixa de:

   menores que 5 e maiores que 200, valores que serão alterados na função distancia() 
   por conta do uso na MAP() esses valores sofreram outra alteração, que pode variar agora, de 0 a 100 
  
 * Sintaxe = map(valor, deMenor, deMaior, paraMenor, paraMaior);
  
    valor       : o número a ser mapeado
    deMenor     : o menor limite do intervalo atual do valor
    deMaior     : o maior limite do intervalo atual do valor
    paraMenor   : o menor limite do intervalo alvo
    paraMaior   : o maior limite do intervalo alvo


    ref: https://www.arduino.cc/reference/pt/language/functions/math/map/
*/
void loop(){
  
  dist2 = map(distancia(sensor2), 5, 200, 0, 100);     // chamando o funcao distancia(), epassando por referencia o sensor2 = sensor Esquerda;   
  dist1 = map(distancia(sensor1), 5, 200, 100, 0);     // chamando o funcao distancia(), epassando por referencia o sensor1 = sensor Direita; 
  robot_forward(velocidade);   // Movendo o carrinho para frente (velocidade e configurada por uma variavel)
/*
 *  Para controle de virada em pista, usar os valores retornados (ainda sem o cauculo? R: Conferir... )com eles, uso de IF e ELSE;
 *  s1 é certeza que e um valor ja cauculado.
*/
microsec_esq = sensor2.timing();
  e2 = sensor2.convert(microsec_esq, Ultrasonic::CM);
microsec_dir = sensor1.timing();
  d1 = sensor1.convert(microsec_dir, Ultrasonic::CM);
    
    if (d1 > 50 or e2 > 50){
    decisao(paraAnalisador (dist1, dist2),50);
         } if (d1 < 20 and e2 < 20){
            parada_emergencia();
            decisao(paraAnalisador(dist1, dist2));             // retorno do cauculo paraconsistente com um valor especifico no S1 que indica o nivel de certeza da colizão;
          }

 
//  Sensores();
}
