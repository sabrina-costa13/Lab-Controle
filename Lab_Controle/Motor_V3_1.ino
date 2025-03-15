#include <PID_v1.h>     //  MODULO DE PID
#include <neotimer.h>
#define TempoLigarMotor 500000 //  Tempo para o motor começar a rodar após liberação de partida (Dado em microsegundos)
#define IntervaloRPM  50000  // Define o valor do intervalo de tempo para calculo do rpm (Dado em microsegundos)
#define IntervaloPID  50000  // Intervalo de processamento do PID
Neotimer parando = Neotimer(1000);// Tempo de 1 segundos para ainda enviar dados após o motor parar

double Setpoint, Input, Output; //  Parametros do PID
PID myPID(&Input, &Output, &Setpoint,0.0053,0.1766,0, DIRECT); // Bloco do PID

// Declaração de pinagem das entradas ou saídas
const int bot1 = 22;      // Giro do motor
const int PINO_IN1 = 9;  // Saída IN1
const int PINO_IN2 = 8;  // Saída IN2
const int PINO_ENA1 = 7; // Saída ENA1
const int pinA = 2;       // Canal A (alterado para a porta 2)

unsigned long tempoUltimaLeituraPrg = 0, tempoDecorridoPrg = 0; //Variáveis usadas para calcular em quanto tempo o programa esta rodando
unsigned long tempoAnteriorRPM,tempoAnteriorPID = 0; // Tempo para calcular a velocidade do motor em rpm e calcular o PID
unsigned long tempoLigaMotor=0; // Variáveis usadas para calcular o tempo para girar o motor
volatile long contadorA = 0;    //Armazena a contagem de pulsos do giro do encoder
unsigned long tempoAtualPrg;

//Parametros para gerar rpm do motor via frequencia com rampa de incremento
unsigned long tempoAtualVel,RampaVel; // Contagem de tempo para velocidade
int instSin = 0;

//Variáveis para uso do estado dos botões
int refMotor0, velmotor0, rpm;
int velmotor1; // Variável de setpoint do motor - Este vindo do seno
int velmotor3; // Variável de setpoint do motor - Controle PID
int rpm_pwm;  
//variáveis para fazer o motor rodar:
bool atraso, ligaMotor, bot1Estado, Ligar = false;


void setup() {
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  pinMode(bot1, INPUT);
  pinMode(pinA, INPUT_PULLUP);
  // Configura a função de interrupção para os pulsos dos canais A e B
  attachInterrupt(digitalPinToInterrupt(pinA), contarPulsoA, RISING);
  Serial.begin(115200);
}

void loop() {
  Input = rpm;
  Setpoint = refMotor0;
  calculaRPM();
  ImprimeStatus();
  controleMotor();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////------SUBROTINAS-----/////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ImprimeStatus(){
  
    //Gera Tempo em µs para geração de gráfico
    Serial.print(micros());
    Serial.print(",");
    //Saida_PWM
    Serial.print(Output);
    Serial.print(",");
    //Referência do motor no instante
    Serial.print(Setpoint);
    Serial.print(",");
    //Velocidade real do motor
    Serial.print(rpm); //input
    Serial.println();

}

void calcularTempoCicloLeitura() {
  // Obtém o tempo atual em microsegundos
  tempoAtualPrg = micros();
  // Calcula o tempo decorrido desde a última leitura
  tempoDecorridoPrg = tempoAtualPrg - tempoUltimaLeituraPrg;
  // Atualize a variável tempoUltimaLeitura com o tempo atual
  tempoUltimaLeituraPrg = tempoAtualPrg;
}

void GiroHorario(){
  analogWrite(PINO_ENA1, Output);
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
    detachInterrupt(digitalPinToInterrupt(pinA));
    rpm = int((contadorA / 70.) / (IntervaloRPM/60000000.));
    tempoAnteriorRPM = micros();
    contadorA = 0;
    attachInterrupt(digitalPinToInterrupt(pinA), contarPulsoA, RISING);
    calculaPID();
  }
}

void calculaPID(){
  if ((micros() - tempoAnteriorPID) > IntervaloPID){

    tempoAnteriorPID = micros();
  }
  myPID.Compute();
}

void controleMotor(){
  if(ligaMotor){
    refMotor0 = map(45,0,90,0,1500); //Referência do motor de 0 a 2040 rpm
  }
  else{
      refMotor0 = 0;
      rpm = 0;
  }
  //bot1Estado = true;
  bot1Estado = digitalRead(bot1);
  if (bot1Estado == 0){
    parando.start();
    atraso = true;
    if(ligaMotor){
      GiroHorario();
    }
    ImprimeStatus();
  }

  if (bot1Estado == 1){
    if (parando.waiting()){
      ImprimeStatus();
    }
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