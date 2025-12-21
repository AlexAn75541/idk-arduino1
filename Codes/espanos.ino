#include <Servo.h>

// --- Configurações gerais ---
int modo = 0; // 0 = Manual, 1 = Automático
int eixo = 0; // 0 = Esquerda-Direita, 1 = Cima-Baixo

// --- Pinos ---
const uint8_t PIN_BOTAO_MODO = 12;   // Botão de Modo (INPUT_PULLUP)
const uint8_t PIN_BOTAO_EIXO = 11;   // Botão de Eixo (INPUT_PULLUP)
const uint8_t PIN_POT = A4;          // Potenciômetro para controle manual
const uint8_t PIN_VOLT = A5;         // Leitura de tensão

// LDRs (sensores de luz)
const uint8_t LDR_SUP_DIR = A0;
const uint8_t LDR_SUP_ESC = A1;
const uint8_t LDR_INF_DIR = A2;
const uint8_t LDR_INF_ESC = A3;

// --- Estados dos botões (debounce simples) ---
int estadoBotaoModo = HIGH;
int estadoBotaoEixo = HIGH;
int estadoBotaoAnteriorModo = HIGH;
int estadoBotaoAnteriorEixo = HIGH;

// --- Leituras dos LDRs ---
int superiorEsquerdo = 0;
int superiorDireito = 0;
int inferiorEsquerdo = 0;
int inferiorDireito = 0;

// --- Servos ---
Servo servoCimaBaixo;
Servo servoEsquerdaDireita;
const uint8_t PIN_SERVO_CIMA = 5;
const uint8_t PIN_SERVO_ESQ = 6;

// --- Parâmetros ---
int valorLimite = 10; // Sensibilidade da medição

void setup() {
  Serial.begin(9600);
  Serial.println("CLEARDATA");
  Serial.println("LABEL,t,voltage,current,power,Mode");

  pinMode(PIN_BOTAO_MODO, INPUT_PULLUP);
  pinMode(PIN_BOTAO_EIXO, INPUT_PULLUP);
  pinMode(PIN_POT, INPUT);

  servoCimaBaixo.attach(PIN_SERVO_CIMA);
  servoEsquerdaDireita.attach(PIN_SERVO_ESQ);

  Serial.println("Setup completo");
}

void loop() {
  // Leitura de tensão/corrente/potência (PLX-DAQ)
  float volt = analogRead(PIN_VOLT) * 5.0 / 1023.0;
  float tensao = 2.0 * volt; // Ajuste do divisor de tensão
  float corrente = tensao / 20.0;
  float potencia = tensao * corrente;

  Serial.print("DATA,TIME,");
  Serial.print(tensao);
  Serial.print(",");
  Serial.print(corrente);
  Serial.print(",");
  Serial.print(potencia);
  Serial.print(",");

  // Tratamento do botão de modo (toggle)
  estadoBotaoModo = digitalRead(PIN_BOTAO_MODO);
  if (estadoBotaoModo != estadoBotaoAnteriorModo) {
    if (estadoBotaoModo == LOW) { // LOW porque pull-up inverte a lógica
      modo = 1 - modo; // alterna entre 0 e 1
      Serial.print("Modo alterado para: ");
      Serial.println(modo == 0 ? "Manual" : "Automático");
    }
  }
  estadoBotaoAnteriorModo = estadoBotaoModo;
  delay(50); // debounce simples

  if (modo == 0) {
    Serial.println("M");
    rastreadorSolarManual();
  } else {
    Serial.println("A");
    rastreadorSolarAutomatico();
  }
}

void rastreadorSolarAutomatico() {
  // Leitura dos LDRs
  superiorDireito = analogRead(LDR_SUP_DIR);
  superiorEsquerdo = analogRead(LDR_SUP_ESC);
  inferiorDireito = analogRead(LDR_INF_DIR);
  inferiorEsquerdo = analogRead(LDR_INF_ESC);

  // Médias por eixo
  int mediaSuperior = (superiorDireito + superiorEsquerdo) / 2;
  int mediaInferior = (inferiorDireito + inferiorEsquerdo) / 2;
  int mediaEsquerda = (superiorEsquerdo + inferiorEsquerdo) / 2;
  int mediaDireita = (superiorDireito + inferiorDireito) / 2;

  // Diferenças para decidir movimentação
  int diferencaElevacao = mediaSuperior - mediaInferior;
  int diferencaAzimute = mediaDireita - mediaEsquerda;

  // Movimento esquerda-direita (azimute)
  if (abs(diferencaAzimute) >= valorLimite) {
    if (diferencaAzimute > 0 && servoEsquerdaDireita.read() < 180) {
      servoEsquerdaDireita.write(servoEsquerdaDireita.read() + 2);
    } else if (diferencaAzimute < 0 && servoEsquerdaDireita.read() > 0) {
      servoEsquerdaDireita.write(servoEsquerdaDireita.read() - 2);
    }
  }

  // Movimento cima-baixo (elevação)
  if (abs(diferencaElevacao) >= valorLimite) {
    if (diferencaElevacao > 0 && servoCimaBaixo.read() < 180) {
      servoCimaBaixo.write(servoCimaBaixo.read() + 2);
    } else if (diferencaElevacao < 0 && servoCimaBaixo.read() > 0) {
      servoCimaBaixo.write(servoCimaBaixo.read() - 2);
    }
  }
}

void rastreadorSolarManual() {
  // Tratamento do botão de eixo (toggle)
  estadoBotaoEixo = digitalRead(PIN_BOTAO_EIXO);
  if (estadoBotaoEixo != estadoBotaoAnteriorEixo) {
    if (estadoBotaoEixo == LOW) { // LOW porque pull-up inverte a lógica
      eixo = 1 - eixo;
      Serial.print("Eixo alterado para: ");
      Serial.println(eixo == 0 ? "Esquerda-Direita" : "Cima-Baixo");
    }
  }
  estadoBotaoAnteriorEixo = estadoBotaoEixo;
  delay(50); // debounce simples

  // Controle via potenciômetro
  int valorPot = analogRead(PIN_POT);
  int posicaoServo = map(valorPot, 0, 1023, 0, 180);

  if (eixo == 0) {
    servoEsquerdaDireita.write(posicaoServo);
    Serial.print("Posição Esquerda-Direita: ");
    Serial.println(posicaoServo);
  } else {
    servoCimaBaixo.write(posicaoServo);
    Serial.print("Posição Cima-Baixo: ");
    Serial.println(posicaoServo);
  }
}
#include <Servo.h>  // Inicialização de variáveis int modo = 0; // 0 para manual, 1 para automático int eixo = 0; // 0 para esquerda-direita, 1 para cima-baixo int estadoBotaoModo = 0; int estadoBotaoEixo = 0; int estadoBotaoAnteriorModo = 0; int estadoBotaoAnteriorEixo = 0;  // Definição dos pinos dos LDRs int ldrSuperiorDireito = A0; int ldrSuperiorEsquerdo = A1; int ldrInferiorDireito = A2; int ldrInferiorEsquerdo = A3;  // Leituras dos LDRs int superiorEsquerdo = 0; int superiorDireito = 0;  int inferiorEsquerdo = 0; int inferiorDireito = 0;  // Declaração dos dois servos Servo servoCimaBaixo; Servo servoEsquerdaDireita;  int valorLimite = 10; // Sensibilidade da medição  void setup() { Serial.begin(9600); Serial.println("CLEARDATA"); Serial.println("LABEL,t,voltage,current,power,Mode");  pinMode(12, INPUT_PULLUP); // Botão de Modo (com pull-up interno) pinMode(11, INPUT_PULLUP); // Botão de Eixo (com pull-up interno) pinMode(A4, INPUT); // Potenciômetro para controle manual  servoCimaBaixo.attach(5); // Servo para movimento cima-baixo servoEsquerdaDireita.attach(6); // Servo para movimento esquerda-direita  Serial.println("Setup completo"); }  void loop() { float volt = analogRead(A5) * 5.0 / 1023; float tensao = 2 * volt; // Tensão=(R1/(R1+R2))*Tensão, R1=R2=10Ohms => tensao=2*volt float corrente = tensao / 20; // I=tensao/(R1+R2) float potencia = tensao * corrente;  Serial.print("DATA,TIME,"); // Comando PLX-DAQ Serial.print(tensao); Serial.print(","); Serial.print(corrente); Serial.print(","); Serial.print(potencia); Serial.print(",");  // Tratamento do botão de modo estadoBotaoModo = digitalRead(12); if (estadoBotaoModo != estadoBotaoAnteriorModo) { if (estadoBotaoModo == LOW) { // LOW porque o pull-up inverte a lógica modo = 1 - modo; // Alterna o modo entre 0 e 1 Serial.print("Modo alterado para: "); Serial.println(modo == 0 ? "Manual" : "Automático"); } } estadoBotaoAnteriorModo = estadoBotaoModo; delay(50); // Atraso para debouncing  if (modo == 0) { Serial.println('M'); rastreadorSolarManual(); } else { Serial.println('A'); rastreadorSolarAutomatico(); } }  void rastreadorSolarAutomatico() { // Captura dos valores analógicos de cada LDR superiorDireito = analogRead(ldrSuperiorDireito); superiorEsquerdo = analogRead(ldrSuperiorEsquerdo); inferiorDireito = analogRead(ldrInferiorDireito); inferiorEsquerdo = analogRead(ldrInferiorEsquerdo);  // Calculando a média int mediaSuperior = (superiorDireito + superiorEsquerdo) / 2; int mediaInferior = (inferiorDireito + inferiorEsquerdo) / 2; int mediaEsquerda = (superiorEsquerdo + inferiorEsquerdo) / 2; int mediaDireita = (superiorDireito + inferiorDireito) / 2;  // Calculando as diferenças int diferencaElevacao = mediaSuperior - mediaInferior; int diferencaAzimute = mediaDireita - mediaEsquerda;  // Movimento esquerda-direita do rastreador solar if (abs(diferencaAzimute) >= valorLimite) { if (diferencaAzimute > 0 && servoEsquerdaDireita.read() < 180) { servoEsquerdaDireita.write(servoEsquerdaDireita.read() + 2); } if (diferencaAzimute < 0 && servoEsquerdaDireita.read() > 0) { servoEsquerdaDireita.write(servoEsquerdaDireita.read() - 2); } }  // Movimento cima-baixo do rastreador solar if (abs(diferencaElevacao) >= valorLimite) { if (diferencaElevacao > 0 && servoCimaBaixo.read() < 180) { servoCimaBaixo.write(servoCimaBaixo.read() + 2); } if (diferencaElevacao < 0 && servoCimaBaixo.read() > 0) { servoCimaBaixo.write(servoCimaBaixo.read() - 2); } } }  void rastreadorSolarManual() { // Tratamento do botão de eixo estadoBotaoEixo = digitalRead(11); if (estadoBotaoEixo != estadoBotaoAnteriorEixo) { if (estadoBotaoEixo == LOW) { // LOW porque o pull-up inverte a lógica eixo = 1 - eixo; // Alterna o eixo entre 0 e 1 Serial.print("Eixo alterado para: "); Serial.println(eixo == 0 ? "Esquerda-Direita" : "Cima-Baixo"); } } estadoBotaoAnteriorEixo = estadoBotaoEixo; delay(50); // Atraso para debouncing  if (eixo == 0) { // Controle esquerda-direita int valorPot = analogRead(A4); int posicaoServo = map(valorPot, 0, 1023, 0, 180); servoEsquerdaDireita.write(posicaoServo); Serial.print("Posição Esquerda-Direita: "); Serial.println(posicaoServo); } else { // Controle cima-baixo int valorPot = analogRead(A4); int posicaoServo = map(valorPot, 0, 1023, 0, 180); servoCimaBaixo.write(posicaoServo); Serial.print("Posição Cima-Baixo: "); Serial.println(posicaoServo); } }