  // --- Bibliotecas Auxiliares ---
#include <AFMotor.h>         //Inclui biblioteca AF Motor
#include <Ultrasonic.h>      //Biblioteca para controle dos Ultrasonicos

// --- Seleção dos Motores ---
AF_DCMotor motor1(1); //Seleção do Motor 1
AF_DCMotor motor2(2); //Seleção do Motor 1
AF_DCMotor motor3(3); //Seleção do Motor 1
AF_DCMotor motor4(4); //Seleção do Motor 1
 
// --- Mapeamento de Hardware ---

#define   trige        A2                 //Saída para o pino de trigger do sensor Esquerda
#define   echoe        A3                 //Entrada para o pino de echo do sensor Esquerda
#define   trigd        A0                 //Saída para o pino de trigger do sensor Direita
#define   echod        A1                 //Entrada para o pino de echo do sensor Direita

                  // --- definição dos sensores --- 
                    // ( As saidas do Shied, todas as 6 (de A0 a A5) podem ser usadas tanto como Analogicas ou digitais, para definição, 
                    // precisamos nomealas pela letra A seguida do digito desejado.
                    
                     #define pinSensor1 A4  // Definição do sensor na porta Analogica 0 (sensor frente)
                     #define pinSensor2 A5  // Definição do sensor na porta Analogica 1 (sensor tras)
                  
                  // --- definição do led --- 
                     
// --- Protótipo das Funções Auxiliares ---
void decision_livre(float microsec_dir, float microsec_esq);                        //Função para tomada de decisão. Qual melhor caminho?
void decision_pista(float microsec_dir, float microsec_esq);                        //Função para tomada de decisão. Qual melhor caminho?
void robot_forward(unsigned char v);    //Função para movimentar robô para frente
void robot_backward(unsigned char v);   //Função para movimentar robô para trás
void robot_left(unsigned char v);       //Função para movimentar robô para esquerda
void robot_right(unsigned char v);      //Função para movimentar robô para direita
void robot_stop(unsigned char v);       //Função para parar o robô
void Sensores();                        //Função dos sensores
 
// --- Variáveis Globais ---
unsigned char velocidade = 0x00;       //Armazena a velocidade dos motores (8 bits)
float dist_cm_direita;                 //Armazena a distância em centímetros entre o robô e o obstáculo a direita
float dist_cm_esquerda;                //Armazena a distância em centímetros entre o robô e o obstáculo a esquerda          

        // testes da nova obtencão de valores ---
        // --------------------------------------------------------------------
              Ultrasonic ultrasonic_esq(trige, echoe);
              Ultrasonic ultrasonic_dir(trigd, echod);
              
              float microsec_dir; 
              float microsec_esq;
        // --------------------------------------------------------------------
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
          
                
  //A biblioteca configura as entradas e saídas pertinentes ao Motor Shield...
    
 pinMode(trige, OUTPUT);                       //Saída para o pulso de trigger
 pinMode(echoe, INPUT);                        //Entrada para o pulso de echo
 pinMode(trigd, OUTPUT);                       //Saída para o pulso de trigger
 pinMode(echod, INPUT);                        //Entrada para o pulso de echo
  
 digitalWrite(trige, LOW);                     //Pino de trigger inicia em low
 digitalWrite(trigd, LOW);                     //Pino de trigger inicia em low
 delay(500);                                   //Aguarda meio segundo antes de iniciar
  velocidade = 255;                             //Controle da velocidade dos motores!
} //end setup
 
// --- Loop Infinito ---
void loop()
{
  Sensores();
     robot_forward(velocidade);   // Movendo o carrinho para frente (velocidade e configurada por uma variavel)
     //delay(10);    // <- Mudado 13/09  (controlando por quantos segundos o carrinho vai se mover antes de começar a localizar obstaculos)
           
            float dir_cmMsec, dir_inMsec;
            microsec_dir = ultrasonic_dir.timing();
     dist_cm_direita = ultrasonic_dir.convert(microsec_dir, Ultrasonic::CM);     // setando na variavel dist_cm os valores obtidos do sensor ultrasonico.
            if(dist_cm_direita != 0){
              Serial.println(dist_cm_direita,DEC);
            }

            float esq_cmMsec, esq_inMsec;
            microsec_esq = ultrasonic_esq.timing();
     dist_cm_esquerda = ultrasonic_esq.convert(microsec_esq, Ultrasonic::CM);   // setando na variavel dist_cm os valores obtidos do sensor ultrasonico.
             if(dist_cm_esquerda != 0){
              Serial.println(dist_cm_esquerda,DEC);
              }
 
     if(dist_cm_direita < 50 || dist_cm_esquerda < 50) // caso algum valor seja obtido e a distância seja menor que 50 cm... faça;
     {
          Serial.println("Vai bater!!");
          //decision_livre(); // chamando a função de decisão;
          decision_pista(microsec_dir,microsec_esq);
     }
} //end loop
 

void decision_livre(float microsec_dir, float microsec_esq)                                                 //Compara as distâncias e decide qual melhor caminho a seguir
{ 
   robot_stop(velocidade);                                      //Para o robô
   delay(150);                                                 //Aguarda 500ms
  
            dist_cm_direita = ultrasonic_dir.convert(microsec_dir, Ultrasonic::CM);                           //Mede distância do sensor mais a direita
   
            dist_cm_esquerda = ultrasonic_esq.convert(microsec_esq, Ultrasonic::CM);                         //Mede distância do sensore mais a esquerda
   
// if 1.0)
   if(dist_cm_direita > dist_cm_esquerda)                       //Distância da direita maior que da esquerda?
   {                                                            //Sim...                                              
      robot_backward(velocidade);                               //Move o robô para trás
      delay(25);                                                //Por 200ms
      robot_right(velocidade);                                  //Move o robô para direita
      delay(500);                                               //Por 2000ms  <- Mudado 13/

            // Medida em cascata (Checando quinas)
            
            microsec_dir = ultrasonic_dir.timing();
            dist_cm_direita = ultrasonic_dir.convert(microsec_dir, Ultrasonic::CM);                            //Mede distância do sensor mais a direita
            
            microsec_esq = ultrasonic_esq.timing();
            dist_cm_esquerda = ultrasonic_esq.convert(microsec_esq, Ultrasonic::CM);                         //Mede distância do sensore mais a esquerda
// if 1.1
                    if(dist_cm_direita > dist_cm_esquerda)                //Distância da esquerda maior que da direita?
                       {                                                  //Sim      
                          robot_backward(velocidade);                     //Move o robô para trás
                          delay(25);             
                          robot_right(velocidade);                        //Move o robô para esquerda
                          delay(20);                                      //Por 2000ms      <- Mudado 13/09
                        } //end if
       robot_forward(velocidade);                                //Move o robô para frente
       } //end if

// if 2.0
    if(dist_cm_direita < dist_cm_esquerda)                      //Distância da esquerda maior que da direita?
   {                                                            //Sim
      robot_backward(velocidade);                               //Move o robô para trás
      delay(25);                                                //Por 200ms
      robot_left(velocidade);                                   //Move o robô para esquerda
      delay(500);   
              
        //Medida em cascata (Checando quinas)            
                microsec_dir = ultrasonic_dir.timing();
                dist_cm_direita = ultrasonic_dir.convert(microsec_dir, Ultrasonic::CM);                             //Mede distância do sensor mais a direita
                
                microsec_esq = ultrasonic_esq.timing();
                dist_cm_esquerda = ultrasonic_esq.convert(microsec_esq, Ultrasonic::CM);                            //Mede distância do sensore mais a esquerda
//if 2.1                
           if(dist_cm_direita < dist_cm_esquerda)               //Distância da esquerda maior que da direita?
              {                                                 //Sim
                robot_backward(velocidade);                     //Move o robô para trás
                delay(25);    
                robot_left(velocidade);                         //Move o robô para esquerda
                delay(20);                                      //Por 2000ms      <- Mudado 13/09
              } //end if
      robot_forward(velocidade);                                //Move o robô para frente
   } //end if   
} //end decision

void decision_pista(float microsec_dir, float microsec_esq)
{
  robot_stop(velocidade);
  delay(300);
                //depois de localizar obstaculos no loop, faz uma nova analise.
                dist_cm_direita = ultrasonic_dir.convert(microsec_dir, Ultrasonic::CM);                             //Mede distância do sensor mais a direita
                
                dist_cm_esquerda = ultrasonic_esq.convert(microsec_esq, Ultrasonic::CM);                         //Mede distância do sensore mais a esquerda

          //tomando decisão de virar para a esquerda.
  if(dist_cm_direita < 50)
      {
        virada_leve_esquerda(velocidade);     //Vira para a esquerda.
        
        virada_leve_direita(velocidade);      //Acerta virando novamente para a direita.
      }

  if(dist_cm_esquerda < 50)
      {
        virada_leve_direita(velocidade);      //Acerta virando novamente para a direita.
        
        virada_leve_esquerda(velocidade);     //Vira para a esquerda.
      }

  if(dist_cm_esquerda < 30 && dist_cm_direita < 30)
      {
        decision_livre(microsec_dir,microsec_esq);
      }
   robot_forward(velocidade);                                //Move o robô para frente
}
 
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

void virada_leve_esquerda (unsigned char v)
{
        motor1.setSpeed(v);
        motor1.run(BACKWARD);
        motor2.setSpeed(v);
        motor2.run(BACKWARD);
        motor3.setSpeed(200);
        motor3.run(FORWARD);
        motor4.setSpeed(200);
        motor4.run(FORWARD);
}

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

void virada_leve_direita (unsigned char v)
{
        motor1.setSpeed(200);
        motor1.run(FORWARD);
        motor2.setSpeed(200);
        motor2.run(FORWARD);
        motor3.setSpeed(v);
        motor3.run(BACKWARD);
        motor4.setSpeed(v);
        motor4.run(BACKWARD);
}
 
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
