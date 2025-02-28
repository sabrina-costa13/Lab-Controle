#include <neotimer.h>
 
#define TempoLigarMotor 0 //  Tempo para o motor começar a rodar após liberação de partida (Dado em microsegundos)
// IntervaloRPM 1000 = Valor para realização da resposta ao degrau unitário
// IntervaloRPM 5000 = Valor para acompanhamento da resposta a frequência
#define IntervaloRPM  4000  // Define o valor do intervalo de tempo para calculo do rpm (Dado em microsegundos)
#define RampaFreq 0.5  // Define o valor de frequencia do ciclo do seno em hz

//Temporizadores para uso no programa
Neotimer parando = Neotimer(1000);  // Tempo de 1 segundos para ainda enviar dados após o motor parar
Neotimer loopDados = Neotimer(10); // Envio dos dados para a tela

// Declaração de pinagem das entradas ou saídas
const int bot1 = 22;      //  Giro do motor
const int PINO_IN1 = 9;  //  Saída IN1
const int PINO_IN2 = 8;  //  Saída IN2
const int PINO_ENA1 = 7;  //  Saída ENA1
const int pinA = 2;       // Canal A (alterado para a porta 2)


unsigned long tempoUltimaLeituraPrg = 0, tempoDecorridoPrg = 0; //Variáveis usadas para calcular em quanto tempo o programa esta rodando
unsigned long tempoAnteriorRPM = 0; // Tempo para calcular a velocidade do motor em rpm
unsigned long tempoLigaMotor=0; // Variáveis usadas para calcular o tempo para girar o motor
volatile long contadorA = 0;    //Armazena a contagem de pulsos do giro do encoder
unsigned long tempoAtualPrg;

//Parametros para gerar rpm do motor via frequencia com rampa de incremento
unsigned long tempoAtualVel,RampaVel; // Contagem de tempo para velocidade
int instSin = 0;

//Variáveis para uso do estado dos botões
int refMotor0, velmotor0, rpm, sensorValueA8;
int velmotor1; // Variável de setpoint do motor - Este vindo do seno
//variáveis para fazer o motor rodar:
bool atraso, ligaMotor, bot1Estado;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// CRIAÇÃO DE ARRAY PARA GERAR SENO //////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const int numPontos = 64; // 64 Define o tamanho do array (quantidade de pontos da função sin)
float senoArray[numPontos]; // Crie um array para armazenar os valores da função sin

void setup() {
  pinMode(bot1, INPUT_PULLUP);
  pinMode(pinA, INPUT);
  loopDados.start();
  // Configura a função de interrupção para os pulsos dos canais A e B
  attachInterrupt(digitalPinToInterrupt(pinA), contarPulsoA, RISING);
  Serial.begin(9600);

  // Preencha o array com valores da função sin
  for (int i = 0; i < numPontos; i++) {
    float fase = 2 * PI * i / numPontos; // Fase varia de 0 a 2*PI
    senoArray[i] = 105 + 35*sin(fase);
  }
  //Cria um print da função seno criada para visualização
  for (int i = 0; i < numPontos; i++) {
    Serial.print("Função_seno:");
    Serial.println(senoArray[i]);
  }
  RampaVel = 1000000/(numPontos*RampaFreq); // Converte o valor de frequencia do ciclo do seno para microsegundos e realizar incrementos no arduino
}

void loop(){
  calcularTempoCicloLeitura();
  calculaRPM();
  if(ligaMotor){
    refMotor0 = sensorValueA8 * (1500 / 1023.0); //Referência do motor de 0 a 2400 rpm //sensorValueA8
    velmotor0 = 140;
  }
  else{
      refMotor0 = 0;
      velmotor0 =0;
  }
  bot1Estado = digitalRead(bot1);
  sensorValueA8 = analogRead(A8); 
  if (bot1Estado == 0){
    parando.start();
    atraso = true;
    if(ligaMotor){
      GiroHorario();
      CalcVel();
    }
    ImprimeStatus();
  }

  if (bot1Estado == 0){
    paraMotor();
    atraso = false;
    ligaMotor = false;
  }

  if(!atraso){
    tempoLigaMotor = micros();
  }
  if(!ligaMotor && ((micros()-tempoLigaMotor) >= TempoLigarMotor)){
    ligaMotor = true;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////------SUBROTINAS-----/////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ImprimeStatus(){
  if(loopDados.done()){
    //Gera Tempo em µs para geração de gráfico
    Serial.print(micros());
    Serial.print(",");
    Serial.print(tempoDecorridoPrg);
    Serial.print(",");
    Serial.print(map(velmotor1, 0, 140, 0, 1500));
    Serial.print(",");
    Serial.print(rpm);
    Serial.println();
    loopDados.start();
  }
}

void calcularTempoCicloLeitura() {
  // Obtém o tempo atual em milissegundos
  tempoAtualPrg = micros();
  // Calcula o tempo decorrido desde a última leitura
  tempoDecorridoPrg = tempoAtualPrg - tempoUltimaLeituraPrg;
  // Atualize a variável tempoUltimaLeitura com o tempo atual
  tempoUltimaLeituraPrg = tempoAtualPrg;
}

void CalcVel(){
  if((micros() - tempoAtualVel) > RampaVel){
    velmotor1 = senoArray[instSin]; 
    instSin++;
    if(instSin == numPontos){
      instSin = 0;
    }
    tempoAtualVel = micros();
  }
}

void GiroHorario(){
  //Utilizando velmotor0 é para resposta ao degrau e velmotor1 para resposta em frequência
  analogWrite(PINO_ENA1, velmotor1);
  digitalWrite(PINO_IN1, HIGH); 
  digitalWrite(PINO_IN2, LOW);
}

void paraMotor(){
  analogWrite(PINO_ENA1, 0);
  digitalWrite(PINO_IN1, HIGH); 
  digitalWrite(PINO_IN2, LOW);
}

void contarPulsoA() {
  contadorA++;
}

void calculaRPM(){
  if ((micros() - tempoAnteriorRPM) > IntervaloRPM){
    rpm = int((contadorA / 70.) / (IntervaloRPM/60000000.));
    tempoAnteriorRPM = micros();
    contadorA = 0;
  }
}