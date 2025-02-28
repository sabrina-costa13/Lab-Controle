#define TempoLigarMotor 500000 // Tempo para o motor começar a rodar após liberação de partida (Dado em microsegundos) - 0.5 segundo
#define IntervaloRPM  40000  // Define o valor do intervalo de tempo para cálculo do rpm (Dado em microsegundos)

// Declaração de pinagem das entradas ou saídas
const int bot1 = 22;      // Giro do motor
const int PINO_IN1 = 9;  // Saída IN1
const int PINO_IN2 = 8;  // Saída IN2
const int PINO_ENA1 = 7; // Saída ENA1
const int pinA = 2;       // Canal A (alterado para a porta 2)

unsigned long tempoUltimaLeituraPrg = 0, tempoDecorridoPrg = 0; // Variáveis usadas para calcular em quanto tempo o programa está rodando
unsigned long tempoAnteriorRPM = 0; // Tempo para calcular a velocidade do motor em rpm
unsigned long tempoLigaMotor = 0; // Variáveis usadas para calcular o tempo para girar o motor
volatile long contadorA = 0;    // Armazena a contagem de pulsos do giro do encoder
unsigned long tempoAtualPrg;

// Variáveis para uso do estado dos botões
int refMotor0, velmotor0, rpm;
bool atraso, ligaMotor, bot1Estado;

void setup() {
  Serial.begin(9600);
  pinMode(bot1, INPUT);
  pinMode(pinA, INPUT_PULLUP); // Pino do encoder
  attachInterrupt(digitalPinToInterrupt(pinA), contarPulsoA, RISING);
}

void loop(){
  calcularTempoCicloLeitura();
  calculaRPM();

  if(ligaMotor){
    refMotor0 = 90; // Referência do motor de 0 a 1500 rpm
    velmotor0 = 45;  // Calcula velocidade do motor de 0 a 100% com base no PWM
  } else {
    refMotor0 = 0;
    velmotor0 = 0;
  }

  bot1Estado = digitalRead(bot1);
  if (bot1Estado == 0){
    atraso = true;
    if(ligaMotor){
      GiroHorario();
    }
    ImprimeStatus();
  }

  if (bot1Estado == 1){
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

void ImprimeStatus(){
  Serial.print(micros());
  Serial.print(",");
  Serial.print(tempoDecorridoPrg);
  Serial.print(",");
  Serial.print(map(refMotor0, 0, 90, 0, 1500));
  Serial.print(",");
  Serial.print(rpm);
  Serial.println();
}

void calcularTempoCicloLeitura() {
  tempoAtualPrg = micros();
  tempoDecorridoPrg = tempoAtualPrg - tempoUltimaLeituraPrg;
  tempoUltimaLeituraPrg = tempoAtualPrg;
}

void GiroHorario(){
  analogWrite(PINO_ENA1, velmotor0);
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