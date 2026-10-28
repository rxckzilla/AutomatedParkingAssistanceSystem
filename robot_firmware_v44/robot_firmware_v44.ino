#include <WiFi.h>

// ======================================================
// V2X IDENTIFICATION
// ======================================================

const char* VEHICLE_ID = "ROBOT";
const char* VEHICLE_TYPE = "AUTONOMOUS";


// ======================================================
// WI-FI AND TCP
// ======================================================

const char* WIFI_SSID = "SET_WIFI_SSID";
const char* WIFI_PASSWORD = "SET_WIFI_PASSWORD";

IPAddress IP_RASPBERRY(188, 188, 1, 4);
const uint16_t PORTA_TCP = 1234;

WiFiClient clienteTCP;


// ======================================================
// HEARTBEAT AND PROTOCOL
// ======================================================

const unsigned long INTERVALO_HEARTBEAT = 5000;
const unsigned long TIMEOUT_SERVIDOR = 20000;

unsigned long ultimoHeartbeat = 0;
unsigned long ultimaMensagemServidor = 0;

bool registadoNoServidor = false;

// Negotiated in the infrastructure's WELCOME message.
// Starting with PROTOCOL=2, reject every instruction without TO.
bool enderecamentoObrigatorio = false;
String bufferTCP;


// ======================================================
// SN754410 PINS
// ======================================================

// Left motor
const int PINO_IN1 = 25;
const int PINO_IN2 = 33;
const int PINO_PWM_ESQUERDO = 13;

// Right motor
const int PINO_IN3 = 26;
const int PINO_IN4 = 27;
const int PINO_PWM_DIREITO = 14;


// ======================================================
// PWM CONFIGURATION
// ======================================================

const int FREQUENCIA_PWM = 500;
const int RESOLUCAO_PWM = 8;


// ======================================================
// FORWARD MOVEMENT
// ======================================================

const int PWM_IMPULSO_ESQUERDO = 180;
const int PWM_IMPULSO_DIREITO = 180;
const unsigned long DURACAO_IMPULSO_FRENTE = 200;

const int PWM_FRENTE_ESQUERDO = 120;
const int PWM_FRENTE_DIREITO = 120;

// After RFID confirms the ROBOT's EXIT, it moves forward to the starting
// position for approximately 2 seconds.
// Reuse the same logic as command F:
// 200 ms at 180/180, followed by 120/120 until 2000 ms have elapsed.
const unsigned long DURACAO_AVANCO_APOS_RFID_SAIDA = 2000;

// Apply this adjustment ONLY after the left turn during SAIR1.
// A slightly higher PWM on the left wheel creates a small rightward tendency
// to compensate for the deviation observed after the turn.
const int PWM_SAIDA_APOS_CURVA_ESQUERDO = 120;
const int PWM_SAIDA_APOS_CURVA_DIREITO = 120;


// ======================================================
// RIGHT TURN
// ======================================================

const int PWM_CURVA_ESQUERDO = 255;
const int PWM_CURVA_DIREITO = 255;

const unsigned long DURACAO_CURVA_DIREITA = 260;

// AUTOMATIC EXIT FROM SPACE 1
// U = approximately 180-degree rotation before leaving space 1.
const unsigned long DURACAO_CURVA_180_SAIDA = 580;

// L = left turn at the corner during exit.
const unsigned long DURACAO_CURVA_ESQUERDA_SAIDA = 285;

const unsigned long PAUSA_ANTES_CURVA = 200;
const unsigned long PAUSA_DEPOIS_CURVA = 180;


// ======================================================
// SAFE PLACE
// ======================================================

const unsigned long DURACAO_AVANCO_SAFE_PLACE = 1500;

// Return from the safe spot: initial reverse movement followed only by the
// main left pivot. No additional alignment corrections.
const unsigned long DURACAO_RECUO_SAFE_PLACE = 1750;
const unsigned long DURACAO_CURVA_ESQUERDA_RETORNO = 320;


// ======================================================
// RECONNECTION
// ======================================================

const unsigned long INTERVALO_RECONEXAO_WIFI = 5000;
const unsigned long INTERVALO_RECONEXAO_TCP = 2000;

unsigned long ultimaTentativaWiFi = 0;
unsigned long ultimaTentativaTCP = 0;


// ======================================================
// MOTOR STATES
// ======================================================

enum EstadoMovimento {
  PARADO,

  IMPULSO_FRENTE,
  FRENTE_LENTA,

  // Final forward movement after RFID confirms exit from the parking area.
  SAIDA_RFID_IMPULSO_FRENTE,
  SAIDA_RFID_FRENTE_LENTA,

  PAUSA_ANTES_DA_CURVA,
  CURVA_DIREITA_ATIVA,
  PAUSA_DEPOIS_DA_CURVA,
  IMPULSO_FRENTE_APOS_CURVA,

  SAFE_PAUSA_ANTES_CURVA,
  SAFE_CURVA_DIREITA,
  SAFE_PAUSA_DEPOIS_CURVA,
  SAFE_AVANCO_LENTO,

  RETORNO_SAFE_RECUO,
  RETORNO_SAFE_PAUSA_ANTES_CURVA,
  RETORNO_SAFE_CURVA_ESQUERDA,
  RETORNO_SAFE_PAUSA_FINAL,
  RETORNO_SAFE_RECUO_FINAL,

  SAIDA_180_PAUSA_ANTES_CURVA,
  SAIDA_180_CURVA_ATIVA,
  SAIDA_180_PAUSA_DEPOIS_CURVA,
  SAIDA_180_IMPULSO_FRENTE,

  SAIDA_ESQUERDA_PAUSA_ANTES_CURVA,
  SAIDA_ESQUERDA_CURVA_ATIVA,
  SAIDA_ESQUERDA_PAUSA_DEPOIS_CURVA,
  SAIDA_ESQUERDA_IMPULSO_FRENTE
};

EstadoMovimento estadoMovimento = PARADO;
unsigned long instanteInicioEstado = 0;

// Preserve the motor state during a temporary interruption
// of the exit. H pauses without losing the U/L/F phase; J resumes its timing.
bool saidaPausada = false;
EstadoMovimento estadoAntesPausa = PARADO;
unsigned long tempoDecorridoAntesPausa = 0;
int in1AntesPausa = LOW, in2AntesPausa = LOW;
int in3AntesPausa = LOW, in4AntesPausa = LOW;
int pwmEsqAntesPausa = 0, pwmDirAntesPausa = 0;


// ======================================================
// TCP TRANSMISSION
// ======================================================

void enviarLinhaTCP(const String& mensagem) {
  if (!clienteTCP.connected()) {
    return;
  }

  clienteTCP.print(mensagem);
  clienteTCP.print('\n');

  Serial.print(">>> RPi: ");
  Serial.println(mensagem);
}


void enviarAck(const String& acao, const String& estado) {
  String mensagem =
    "ACK|ID=" + String(VEHICLE_ID) +
    "|ACTION=" + acao +
    "|STATUS=" + estado;

  enviarLinhaTCP(mensagem);
}


void enviarEstado(const String& estado, const String& localizacao) {
  String mensagem =
    "STATUS|ID=" + String(VEHICLE_ID) +
    "|STATE=" + estado +
    "|LOCATION=" + localizacao;

  enviarLinhaTCP(mensagem);
}


// ======================================================
// MOTOR FUNCTIONS
// ======================================================

void definirPWM(int pwmEsquerdo, int pwmDireito) {
  pwmEsquerdo = constrain(pwmEsquerdo, 0, 255);
  pwmDireito = constrain(pwmDireito, 0, 255);

  ledcWrite(PINO_PWM_ESQUERDO, pwmEsquerdo);
  ledcWrite(PINO_PWM_DIREITO, pwmDireito);
}


void pararMotores() {
  definirPWM(0, 0);

  digitalWrite(PINO_IN1, LOW);
  digitalWrite(PINO_IN2, LOW);

  digitalWrite(PINO_IN3, LOW);
  digitalWrite(PINO_IN4, LOW);
}


void configurarMotoresFrente() {
  digitalWrite(PINO_IN1, HIGH);
  digitalWrite(PINO_IN2, LOW);

  digitalWrite(PINO_IN3, HIGH);
  digitalWrite(PINO_IN4, LOW);
}


void configurarCurvaDireitaAtiva() {
  digitalWrite(PINO_IN1, HIGH);
  digitalWrite(PINO_IN2, LOW);

  digitalWrite(PINO_IN3, LOW);
  digitalWrite(PINO_IN4, HIGH);
}


void configurarMotoresRe() {
  digitalWrite(PINO_IN1, LOW);
  digitalWrite(PINO_IN2, HIGH);

  digitalWrite(PINO_IN3, LOW);
  digitalWrite(PINO_IN4, HIGH);
}


void configurarCurvaEsquerdaAtiva() {
  digitalWrite(PINO_IN1, LOW);
  digitalWrite(PINO_IN2, HIGH);

  digitalWrite(PINO_IN3, HIGH);
  digitalWrite(PINO_IN4, LOW);
}


// ======================================================
// MOVEMENT COMMANDS
// ======================================================

void iniciarMovimentoFrente() {
  configurarMotoresFrente();

  definirPWM(
    PWM_IMPULSO_ESQUERDO,
    PWM_IMPULSO_DIREITO
  );

  estadoMovimento = IMPULSO_FRENTE;
  instanteInicioEstado = millis();

  Serial.println("F: impulso inicial em frente.");
  enviarAck("FORWARD", "RECEIVED");
}


void pausarSaidaParaConflito() {
  if (saidaPausada) {
    enviarAck("PAUSE_EXIT", "COMPLETED");
    return;
  }
  estadoAntesPausa = estadoMovimento;
  tempoDecorridoAntesPausa = millis() - instanteInicioEstado;
  in1AntesPausa = digitalRead(PINO_IN1);
  in2AntesPausa = digitalRead(PINO_IN2);
  in3AntesPausa = digitalRead(PINO_IN3);
  in4AntesPausa = digitalRead(PINO_IN4);
  pwmEsqAntesPausa = ledcRead(PINO_PWM_ESQUERDO);
  pwmDirAntesPausa = ledcRead(PINO_PWM_DIREITO);
  pararMotores();
  estadoMovimento = PARADO;
  saidaPausada = true;
  Serial.println("Saida suspensa; fase e temporizador guardados.");
  enviarAck("PAUSE_EXIT", "COMPLETED");
}

void retomarSaidaSuspensa() {
  if (!saidaPausada) {
    enviarAck("RESUME_EXIT", "NOT_PAUSED");
    return;
  }
  // Restore direction first, then PWM. Time spent in the
  // SAFE PLACE does not reduce the remaining turn duration.
  digitalWrite(PINO_IN1, in1AntesPausa);
  digitalWrite(PINO_IN2, in2AntesPausa);
  digitalWrite(PINO_IN3, in3AntesPausa);
  digitalWrite(PINO_IN4, in4AntesPausa);
  definirPWM(pwmEsqAntesPausa, pwmDirAntesPausa);
  estadoMovimento = estadoAntesPausa;
  instanteInicioEstado = millis() - tempoDecorridoAntesPausa;
  saidaPausada = false;
  Serial.println("Saida retomada do mesmo estado de movimento.");
  enviarAck("RESUME_EXIT", "COMPLETED");
}

void iniciarAvancoAposRfidSaida() {
  // Avoid restarting the timer if the infrastructure sends
  // EXIT_CONFIRMED immediately followed by the dedicated X command.
  if (
    estadoMovimento == SAIDA_RFID_IMPULSO_FRENTE
    || estadoMovimento == SAIDA_RFID_FRENTE_LENTA
  ) {
    Serial.println(
      "Avanco pos-RFID ja em curso; pedido duplicado ignorado."
    );
    return;
  }

  // The ROBOT is stopped at the entrance after RFID
  // has confirmed that it is leaving the parking area.
  pararMotores();

  configurarMotoresFrente();

  // Use the same PWM values as the normal F command's initial impulse.
  definirPWM(
    PWM_IMPULSO_ESQUERDO,
    PWM_IMPULSO_DIREITO
  );

  estadoMovimento = SAIDA_RFID_IMPULSO_FRENTE;
  instanteInicioEstado = millis();

  Serial.println(
    "RFID de saida confirmado: avanco final para a partida iniciado."
  );
  Serial.println(
    "Primeiros 200 ms a 180/180; depois 120/120 ate completar 2000 ms."
  );

  enviarEstado("EXITING_TO_START", "OUTSIDE");
}


void iniciarCurvaDireita() {
  pararMotores();

  estadoMovimento = PAUSA_ANTES_DA_CURVA;
  instanteInicioEstado = millis();

  Serial.println("R: preparação da curva para o Lugar 1.");
  enviarAck("TURN_RIGHT", "RECEIVED");
}


void iniciarVolta180Saida() {
  pararMotores();

  estadoMovimento = SAIDA_180_PAUSA_ANTES_CURVA;
  instanteInicioEstado = millis();

  Serial.println(
    "U: saída do Lugar 1 iniciada; preparação para rotação ~180 graus."
  );
  enviarAck("TURN_AROUND_EXIT", "RECEIVED");
}


void iniciarCurvaEsquerdaSaida() {
  pararMotores();

  estadoMovimento = SAIDA_ESQUERDA_PAUSA_ANTES_CURVA;
  instanteInicioEstado = millis();

  Serial.println(
    "L: preparação da curva para a esquerda durante a saída."
  );
  enviarAck("TURN_LEFT_EXIT", "RECEIVED");
}


void iniciarSafePlace() {
  pararMotores();

  estadoMovimento = SAFE_PAUSA_ANTES_CURVA;
  instanteInicioEstado = millis();

  Serial.println("P: manobra para o safe place iniciada.");
  enviarAck("SAFE_PLACE", "RECEIVED");
}



void iniciarRetornoSafePlace() {
  pararMotores();

  configurarMotoresRe();
  definirPWM(
    PWM_FRENTE_ESQUERDO,
    PWM_FRENTE_DIREITO
  );

  estadoMovimento = RETORNO_SAFE_RECUO;
  instanteInicioEstado = millis();

  Serial.println(
    "B: regresso do safe place iniciado."
  );
  enviarAck(
    "RETURN_FROM_SAFE_PLACE",
    "RECEIVED"
  );
}

void pararVeiculo() {
  bool estavaEmRetorno = (
    estadoMovimento == RETORNO_SAFE_RECUO
    || estadoMovimento == RETORNO_SAFE_PAUSA_ANTES_CURVA
    || estadoMovimento == RETORNO_SAFE_CURVA_ESQUERDA
    || estadoMovimento == RETORNO_SAFE_PAUSA_FINAL
    || estadoMovimento == RETORNO_SAFE_RECUO_FINAL
  );

  pararMotores();

  estadoMovimento = PARADO;
  instanteInicioEstado = millis();

  Serial.println("S: veículo parado.");
  enviarAck("STOP", "COMPLETED");

  if (estavaEmRetorno) {
    Serial.println(
      "Robot parado na entrada por ordem do sensor da infraestrutura."
    );
    enviarAck(
      "RETURN_FROM_SAFE_PLACE",
      "COMPLETED"
    );
    enviarEstado(
      "WAITING",
      "ENTRANCE"
    );
  }
}


// ======================================================
// MOTOR STATE MACHINE
// ======================================================

void atualizarMovimento() {
  unsigned long agora = millis();
  unsigned long tempoEstado = agora - instanteInicioEstado;

  switch (estadoMovimento) {

    case PARADO:
      break;

    case IMPULSO_FRENTE:
      if (tempoEstado >= DURACAO_IMPULSO_FRENTE) {
        configurarMotoresFrente();

        definirPWM(
          PWM_FRENTE_ESQUERDO,
          PWM_FRENTE_DIREITO
        );

        estadoMovimento = FRENTE_LENTA;
        instanteInicioEstado = agora;

        Serial.println("Avanço lento iniciado.");
        enviarAck("FORWARD", "EXECUTING");
      }
      break;

    case FRENTE_LENTA:
      break;

    case SAIDA_RFID_IMPULSO_FRENTE:
      if (tempoEstado >= DURACAO_IMPULSO_FRENTE) {
        configurarMotoresFrente();

        definirPWM(
          PWM_FRENTE_ESQUERDO,
          PWM_FRENTE_DIREITO
        );

        estadoMovimento = SAIDA_RFID_FRENTE_LENTA;
        instanteInicioEstado = agora;

        Serial.println(
          "Saida RFID: impulso concluido; avanco normal 120/120."
        );
      }
      break;

    case SAIDA_RFID_FRENTE_LENTA:
      if (
        tempoEstado >=
        (DURACAO_AVANCO_APOS_RFID_SAIDA - DURACAO_IMPULSO_FRENTE)
      ) {
        pararMotores();

        estadoMovimento = PARADO;
        instanteInicioEstado = agora;

        Serial.println(
          "Saida RFID: chegada a posicao de partida; ROBOT parado."
        );

        // Keep the logical location as OUTSIDE to allow,
        // for example, another START command later.
        enviarEstado("OUTSIDE", "OUTSIDE");
        enviarAck("EXIT_TO_START", "COMPLETED");
      }
      break;

    case PAUSA_ANTES_DA_CURVA:
      if (tempoEstado >= PAUSA_ANTES_CURVA) {
        configurarCurvaDireitaAtiva();

        definirPWM(
          PWM_CURVA_ESQUERDO,
          PWM_CURVA_DIREITO
        );

        estadoMovimento = CURVA_DIREITA_ATIVA;
        instanteInicioEstado = agora;

        Serial.println("Curva ativa para a direita.");
      }
      break;

    case CURVA_DIREITA_ATIVA:
      if (tempoEstado >= DURACAO_CURVA_DIREITA) {
        pararMotores();

        estadoMovimento = PAUSA_DEPOIS_DA_CURVA;
        instanteInicioEstado = agora;

        Serial.println("Curva concluída.");
      }
      break;

    case PAUSA_DEPOIS_DA_CURVA:
      if (tempoEstado >= PAUSA_DEPOIS_CURVA) {
        configurarMotoresFrente();

        definirPWM(
          PWM_IMPULSO_ESQUERDO,
          PWM_IMPULSO_DIREITO
        );

        estadoMovimento = IMPULSO_FRENTE_APOS_CURVA;
        instanteInicioEstado = agora;

        Serial.println("Impulso em direção ao Lugar 1.");
      }
      break;

    case IMPULSO_FRENTE_APOS_CURVA:
      if (tempoEstado >= DURACAO_IMPULSO_FRENTE) {
        configurarMotoresFrente();

        definirPWM(
          PWM_FRENTE_ESQUERDO,
          PWM_FRENTE_DIREITO
        );

        estadoMovimento = FRENTE_LENTA;
        instanteInicioEstado = agora;

        Serial.println("Avanço lento para o Lugar 1.");
        enviarAck("TURN_RIGHT", "COMPLETED");
      }
      break;

    case SAIDA_180_PAUSA_ANTES_CURVA:
      if (tempoEstado >= PAUSA_ANTES_CURVA) {
        configurarCurvaDireitaAtiva();

        definirPWM(
          PWM_CURVA_ESQUERDO,
          PWM_CURVA_DIREITO
        );

        estadoMovimento = SAIDA_180_CURVA_ATIVA;
        instanteInicioEstado = agora;

        Serial.println(
          "SAIR1: rotação contínua ~180 graus iniciada."
        );
      }
      break;

    case SAIDA_180_CURVA_ATIVA:
      if (tempoEstado >= DURACAO_CURVA_180_SAIDA) {
        pararMotores();

        estadoMovimento = SAIDA_180_PAUSA_DEPOIS_CURVA;
        instanteInicioEstado = agora;

        Serial.println(
          "SAIR1: rotação ~180 graus concluída."
        );
      }
      break;

    case SAIDA_180_PAUSA_DEPOIS_CURVA:
      if (tempoEstado >= PAUSA_DEPOIS_CURVA) {
        configurarMotoresFrente();

        definirPWM(
          PWM_IMPULSO_ESQUERDO,
          PWM_IMPULSO_DIREITO
        );

        estadoMovimento = SAIDA_180_IMPULSO_FRENTE;
        instanteInicioEstado = agora;

        Serial.println(
          "SAIR1: impulso em frente em direção ao canto."
        );
      }
      break;

    case SAIDA_180_IMPULSO_FRENTE:
      if (tempoEstado >= DURACAO_IMPULSO_FRENTE) {
        configurarMotoresFrente();

        definirPWM(
          PWM_FRENTE_ESQUERDO,
          PWM_FRENTE_DIREITO
        );

        estadoMovimento = FRENTE_LENTA;
        instanteInicioEstado = agora;

        Serial.println(
          "SAIR1: avanço lento até ao sensor do canto."
        );
        enviarAck("TURN_AROUND_EXIT", "COMPLETED");
        enviarEstado("EXITING", "TO_CORNER");
      }
      break;

    case SAIDA_ESQUERDA_PAUSA_ANTES_CURVA:
      if (tempoEstado >= PAUSA_ANTES_CURVA) {
        configurarCurvaEsquerdaAtiva();

        definirPWM(
          PWM_CURVA_ESQUERDO,
          PWM_CURVA_DIREITO
        );

        estadoMovimento = SAIDA_ESQUERDA_CURVA_ATIVA;
        instanteInicioEstado = agora;

        Serial.println(
          "SAIR1: curva ativa para a esquerda no canto."
        );
      }
      break;

    case SAIDA_ESQUERDA_CURVA_ATIVA:
      if (tempoEstado >= DURACAO_CURVA_ESQUERDA_SAIDA) {
        pararMotores();

        estadoMovimento = SAIDA_ESQUERDA_PAUSA_DEPOIS_CURVA;
        instanteInicioEstado = agora;

        Serial.println(
          "SAIR1: curva para a esquerda concluída."
        );
      }
      break;

    case SAIDA_ESQUERDA_PAUSA_DEPOIS_CURVA:
      if (tempoEstado >= PAUSA_DEPOIS_CURVA) {
        configurarMotoresFrente();

        definirPWM(
          PWM_IMPULSO_ESQUERDO,
          PWM_IMPULSO_DIREITO
        );

        estadoMovimento = SAIDA_ESQUERDA_IMPULSO_FRENTE;
        instanteInicioEstado = agora;

        Serial.println(
          "SAIR1: impulso em frente em direção à entrada."
        );
      }
      break;

    case SAIDA_ESQUERDA_IMPULSO_FRENTE:
      if (tempoEstado >= DURACAO_IMPULSO_FRENTE) {
        configurarMotoresFrente();

        // Slight rightward adjustment along the Corner -> Entrance segment.
        // Only this SAIR1 phase uses these values.
        definirPWM(
          PWM_SAIDA_APOS_CURVA_ESQUERDO,
          PWM_SAIDA_APOS_CURVA_DIREITO
        );

        estadoMovimento = FRENTE_LENTA;
        instanteInicioEstado = agora;

        Serial.println(
          "SAIR1: avanço lento até ao sensor da entrada."
        );
        enviarAck("TURN_LEFT_EXIT", "COMPLETED");
        enviarEstado("EXITING", "TO_ENTRANCE");
      }
      break;

    case SAFE_PAUSA_ANTES_CURVA:
      if (tempoEstado >= PAUSA_ANTES_CURVA) {
        configurarCurvaDireitaAtiva();

        definirPWM(
          PWM_CURVA_ESQUERDO,
          PWM_CURVA_DIREITO
        );

        estadoMovimento = SAFE_CURVA_DIREITA;
        instanteInicioEstado = agora;

        Serial.println("Safe place: curva ativa.");
      }
      break;

    case SAFE_CURVA_DIREITA:
      if (tempoEstado >= DURACAO_CURVA_DIREITA) {
        pararMotores();

        estadoMovimento = SAFE_PAUSA_DEPOIS_CURVA;
        instanteInicioEstado = agora;

        Serial.println("Safe place: curva concluída.");
      }
      break;

    case SAFE_PAUSA_DEPOIS_CURVA:
      if (tempoEstado >= PAUSA_DEPOIS_CURVA) {
        configurarMotoresFrente();

        definirPWM(
          PWM_FRENTE_ESQUERDO,
          PWM_FRENTE_DIREITO
        );

        estadoMovimento = SAFE_AVANCO_LENTO;
        instanteInicioEstado = agora;

        Serial.println(
          "Safe place: avanço lento durante 1,5 segundos."
        );
      }
      break;

    case SAFE_AVANCO_LENTO:
      if (tempoEstado >= DURACAO_AVANCO_SAFE_PLACE) {
        pararMotores();

        estadoMovimento = PARADO;
        instanteInicioEstado = agora;

        Serial.println("Robot parado no safe place.");
        enviarAck("SAFE_PLACE", "COMPLETED");
        enviarEstado("SAFE_PLACE", "SAFE_PLACE");
      }
      break;

    case RETORNO_SAFE_RECUO:
      if (tempoEstado >= DURACAO_RECUO_SAFE_PLACE) {
        pararMotores();

        estadoMovimento = RETORNO_SAFE_PAUSA_ANTES_CURVA;
        instanteInicioEstado = agora;

        Serial.println(
          "Regresso: recuo adicional concluído; "
          "a iniciar curva inversa."
        );
      }
      break;

    case RETORNO_SAFE_PAUSA_ANTES_CURVA:
      if (tempoEstado >= PAUSA_ANTES_CURVA) {
        configurarCurvaEsquerdaAtiva();

        definirPWM(
          PWM_CURVA_ESQUERDO,
          PWM_CURVA_DIREITO
        );

        estadoMovimento = RETORNO_SAFE_CURVA_ESQUERDA;
        instanteInicioEstado = agora;

        Serial.println(
          "Regresso: curva ativa para a esquerda."
        );
      }
      break;

    case RETORNO_SAFE_CURVA_ESQUERDA:
      if (
        tempoEstado
        >= DURACAO_CURVA_ESQUERDA_RETORNO
      ) {
        pararMotores();

        estadoMovimento = RETORNO_SAFE_PAUSA_FINAL;
        instanteInicioEstado = agora;

        Serial.println(
          "Regresso: curva para a esquerda concluída."
        );
      }
      break;

    case RETORNO_SAFE_PAUSA_FINAL:
      if (tempoEstado >= PAUSA_DEPOIS_CURVA) {
        // After the main 320 ms turn and the 180 ms pause,
        // start the final reverse movement directly, with no additional pivots.
        configurarMotoresRe();

        definirPWM(
          PWM_FRENTE_ESQUERDO,
          PWM_FRENTE_DIREITO
        );

        estadoMovimento = RETORNO_SAFE_RECUO_FINAL;
        instanteInicioEstado = agora;

        Serial.println(
          "Regresso: curva principal concluida; "
          "recuo final 120/120 ate ao sensor da entrada."
        );
        enviarEstado(
          "RETURNING_TO_ENTRANCE",
          "APPROACHING_ENTRANCE"
        );
      }
      break;

    case RETORNO_SAFE_RECUO_FINAL:
      // Continuous reverse movement at 120/120.
      // Stop only when the infrastructure sends command S
      // after detecting the ROBOT at the entrance sensor.
      break;
  }
}


// ======================================================
// WI-FI
// ======================================================

void tentarLigarWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  unsigned long agora = millis();

  if (
    agora - ultimaTentativaWiFi
    < INTERVALO_RECONEXAO_WIFI
  ) {
    return;
  }

  ultimaTentativaWiFi = agora;

  Serial.print("A ligar ao Wi-Fi: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}


void verificarWiFi() {
  static bool estavaLigado = false;

  bool ligadoAgora = WiFi.status() == WL_CONNECTED;

  if (ligadoAgora && !estavaLigado) {
    Serial.println("Wi-Fi ligado.");

    Serial.print("IP do ESP32: ");
    Serial.println(WiFi.localIP());
  }

  if (!ligadoAgora && estavaLigado) {
    Serial.println("Wi-Fi desligado.");

    clienteTCP.stop();
    registadoNoServidor = false;
    enderecamentoObrigatorio = false;
    bufferTCP = "";
    pararMotores();
    estadoMovimento = PARADO;
  }

  estavaLigado = ligadoAgora;

  if (!ligadoAgora) {
    tentarLigarWiFi();
  }
}


// ======================================================
// TCP CONNECTION AND REGISTRATION
// ======================================================

void enviarHello() {
  String mensagem =
    "HELLO|ID=" + String(VEHICLE_ID) +
    "|TYPE=" + String(VEHICLE_TYPE) +
    "|PROTOCOL=2";

  enviarLinhaTCP(mensagem);
}


void tentarLigarTCP() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (clienteTCP.connected()) {
    return;
  }

  unsigned long agora = millis();

  if (
    agora - ultimaTentativaTCP
    < INTERVALO_RECONEXAO_TCP
  ) {
    return;
  }

  ultimaTentativaTCP = agora;

  Serial.print("A ligar à Raspberry Pi em ");
  Serial.print(IP_RASPBERRY);
  Serial.print(":");
  Serial.println(PORTA_TCP);

  clienteTCP.stop();

  if (clienteTCP.connect(IP_RASPBERRY, PORTA_TCP)) {
    Serial.println("Ligação TCP estabelecida.");

    registadoNoServidor = false;
    enderecamentoObrigatorio = false;
    bufferTCP = "";
    ultimaMensagemServidor = millis();
    ultimoHeartbeat = millis();

    enviarHello();
  } else {
    Serial.println("Falha na ligação TCP.");
  }
}


void verificarTCP() {
  static bool estavaLigado = false;

  bool ligadoAgora = clienteTCP.connected();

  if (ligadoAgora && !estavaLigado) {
    Serial.println("TCP ligado.");
  }

  if (!ligadoAgora && estavaLigado) {
    Serial.println("TCP desligado.");

    clienteTCP.stop();
    registadoNoServidor = false;
    enderecamentoObrigatorio = false;
    bufferTCP = "";

    pararMotores();
    estadoMovimento = PARADO;
  }

  estavaLigado = ligadoAgora;

  if (!ligadoAgora) {
    tentarLigarTCP();
  }
}


// ======================================================
// MESSAGE PARSING
// ======================================================

String obterCampo(
  const String& mensagem,
  const String& campo
) {
  String procura = campo + "=";
  int inicio = mensagem.indexOf(procura);

  if (inicio < 0) {
    return "";
  }

  inicio += procura.length();

  int fim = mensagem.indexOf('|', inicio);

  if (fim < 0) {
    fim = mensagem.length();
  }

  return mensagem.substring(inicio, fim);
}


void processarComandoMovimento(char comando) {
  comando = toupper(comando);

  switch (comando) {

    case 'F':
      Serial.println("Comando recebido: F");
      iniciarMovimentoFrente();
      break;

    case 'R':
      Serial.println("Comando recebido: R");
      iniciarCurvaDireita();
      break;

    case 'U':
      Serial.println("Comando recebido: U");
      iniciarVolta180Saida();
      break;

    case 'L':
      Serial.println("Comando recebido: L");
      iniciarCurvaEsquerdaSaida();
      break;

    case 'P':
      Serial.println("Comando recebido: P");
      iniciarSafePlace();
      break;

    case 'B':
      Serial.println("Comando recebido: B");
      iniciarRetornoSafePlace();
      break;

    case 'S':
      Serial.println("Comando recebido: S");
      saidaPausada = false;
      pararVeiculo();
      break;

    case 'X':
      Serial.println(
        "Comando recebido: X - avanco final apos RFID de saida"
      );
      iniciarAvancoAposRfidSaida();
      break;

    case 'H':
      pausarSaidaParaConflito();
      break;

    case 'J':
      retomarSaidaSuspensa();
      break;

    default:
      Serial.print("Comando de movimento desconhecido: ");
      Serial.println(comando);
      break;
  }
}


void processarEvento(const String& mensagem) {
  String tipoEvento = obterCampo(mensagem, "TYPE");

  Serial.print("Evento da infraestrutura: ");
  Serial.println(tipoEvento);

  if (tipoEvento == "ENTRY_AUTHORIZED") {
    enviarEstado("ENTRY_AUTHORIZED", "ENTRANCE");
  } else if (tipoEvento == "PARKING_COMPLETED") {
    enviarEstado("PARKED", "PLACE_1");
  } else if (tipoEvento == "EXIT_CONFIRMED") {
    // The Raspberry Pi sends this event only after validating
    // the ROBOT's exit RFID. In any ROBOT exit scenario,
    // then execute the final forward movement to the starting position.
    enviarEstado("OUTSIDE", "OUTSIDE");
    iniciarAvancoAposRfidSaida();
  } else if (tipoEvento == "RETURN_FROM_SAFE_PLACE") {
    Serial.println(
      "Saída do Carro 2 confirmada. "
      "Regresso do safe place autorizado."
    );
  } else if (tipoEvento == "RETURN_COMPLETED") {
    Serial.println(
      "Posição inicial confirmada pela infraestrutura."
    );
  } else if (tipoEvento == "WAIT_AT_CORNER") {
    Serial.println(
      "Carro 2 detetado na entrada. "
      "Robot deve aguardar no canto."
    );
    enviarEstado("WAITING_CLEAR_PATH", "CORNER");
    enviarAck("WAIT_AT_CORNER", "RECEIVED");
  } else if (tipoEvento == "EXIT_CONTINUE") {
    Serial.println(
      "Entrada livre. Robot autorizado a avancar "
      "ate ao leitor RFID de saida."
    );
    enviarEstado("EXITING", "CORNER");
    enviarAck("EXIT_CONTINUE", "RECEIVED");
  } else if (tipoEvento == "RFID_EXIT_REQUIRED") {
    enviarEstado("WAITING_RFID_EXIT", "ENTRANCE");
  } else if (tipoEvento == "SYSTEM_STATE") {
    String estado = obterCampo(mensagem, "STATE");

    Serial.print("Estado atual do parque: ");
    Serial.println(estado);
  }
}


void processarV2V(const String& mensagem) {
  String origem = obterCampo(mensagem, "FROM");
  String evento = obterCampo(mensagem, "EVENT");

  Serial.print("V2V via infraestrutura | origem=");
  Serial.print(origem);
  Serial.print(" | evento=");
  Serial.println(evento);

  if (evento == "CAR_2_EXITING") {
    Serial.println(
      "Informação recebida: Carro 2 iniciou a saída."
    );
  } else if (evento == "CLEAR_PATH_REQUIRED") {
    Serial.println(
      "Infraestrutura poderá ordenar a manobra P."
    );
  } else if (evento == "CAR_2_PARKED") {
    Serial.println(
      "Informação recebida: Carro 2 estacionado."
    );
  } else if (evento == "CAR_2_EXITED") {
    Serial.println(
      "Informação recebida: Carro 2 saiu do parque."
    );
  } else if (evento == "VEHICLE_CONNECTED") {
    Serial.println(
      "Outro veículo ligado à infraestrutura."
    );
  } else if (evento == "VEHICLE_DISCONNECTED") {
    Serial.println(
      "Outro veículo desligado da infraestrutura."
    );
  }

  enviarAck("V2V_" + evento, "RECEIVED");
}


void processarLinhaTCP(String mensagem) {
  mensagem.trim();

  if (mensagem.length() == 0) {
    return;
  }

  String destinoMensagem = obterCampo(mensagem, "TO");
  if (destinoMensagem.length() > 0) {
    if (destinoMensagem != VEHICLE_ID && destinoMensagem != "ALL") {
      // Received through logical broadcast but addressed to another vehicle.
      // To demonstrate addressing, show only ignored commands
      // on Serial to avoid PONG/EVENT message spam.
      if (mensagem.startsWith("COMMAND|")) {
        Serial.print("BROADCAST: comando para ");
        Serial.print(destinoMensagem);
        Serial.println(" ignorado (ID diferente).");
      }
      // Do not execute, send an ACK, or change local state.
      return;
    }
  } else if (enderecamentoObrigatorio) {
    // With PROTOCOL=2, even WELCOME/PONG/ERROR responses include TO.
    // Reject unaddressed frames, including single-character commands,
    // to prevent unintended actions.
    return;
  }

  // TO=ALL is valid for global events. A global movement command
  // is accepted only when explicitly specified as STOP.
  if (mensagem.startsWith("COMMAND|") && destinoMensagem == "ALL") {
    if (obterCampo(mensagem, "ACTION") != "STOP") {
      Serial.println("COMMAND TO=ALL recusado: apenas STOP global.");
      return;
    }
  }

  if (mensagem.length() == 0) {
    return;
  }

  ultimaMensagemServidor = millis();

  Serial.print("<<< RPi: ");
  Serial.println(mensagem);

  // Direct compatibility with legacy commands.
  if (mensagem.length() == 1) {
    char comando = mensagem.charAt(0);

    if (
      comando == 'F'
      || comando == 'f'
      || comando == 'S'
      || comando == 's'
      || comando == 'R'
      || comando == 'r'
      || comando == 'U'
      || comando == 'u'
      || comando == 'L'
      || comando == 'l'
      || comando == 'P'
      || comando == 'B'
      || comando == 'p'
    ) {
      processarComandoMovimento(comando);
      return;
    }
  }

  if (mensagem.startsWith("WELCOME|")) {
    String idRecebido = obterCampo(mensagem, "ID");

    if (idRecebido == VEHICLE_ID) {
      enderecamentoObrigatorio = (obterCampo(mensagem, "PROTOCOL") == "2");
      registadoNoServidor = true;

      Serial.println(
        "Robot registado no servidor multi-cliente."
      );

      enviarEstado("CONNECTED", "UNKNOWN");
    }
    return;
  }

  if (mensagem.startsWith("PONG|")) {
    return;
  }

  if (mensagem.startsWith("EVENT|")) {
    processarEvento(mensagem);
    return;
  }

  if (mensagem.startsWith("V2V|")) {
    processarV2V(mensagem);
    return;
  }

  if (mensagem.startsWith("ERROR|")) {
    Serial.print("Erro comunicado pelo servidor: ");
    Serial.println(mensagem);
    return;
  }

  // Future support for structured commands.
  if (mensagem.startsWith("COMMAND|")) {
    String acao = obterCampo(mensagem, "ACTION");

    if (acao == "FORWARD") {
      processarComandoMovimento('F');
    } else if (acao == "STOP") {
      processarComandoMovimento('S');
    } else if (acao == "TURN_RIGHT") {
      processarComandoMovimento('R');
    } else if (acao == "TURN_AROUND_EXIT") {
      processarComandoMovimento('U');
    } else if (acao == "TURN_LEFT_EXIT") {
      processarComandoMovimento('L');
    } else if (acao == "SAFE_PLACE") {
      processarComandoMovimento('P');
    } else if (acao == "RETURN_FROM_SAFE_PLACE") {
      processarComandoMovimento('B');
    } else if (acao == "EXIT_TO_START") {
      processarComandoMovimento('X');
    } else if (acao == "PAUSE_EXIT") {
      processarComandoMovimento('H');
    } else if (acao == "RESUME_EXIT") {
      processarComandoMovimento('J');
    } else {
      enviarAck(acao, "UNKNOWN_ACTION");
    }

    return;
  }

  Serial.print("Mensagem não reconhecida: ");
  Serial.println(mensagem);
}


// ======================================================
// LINE-BASED TCP RECEPTION
// ======================================================

void lerMensagensTCP() {
  while (
    clienteTCP.connected()
    && clienteTCP.available()
  ) {
    char caractere = clienteTCP.read();

    if (caractere == '\r') {
      continue;
    }

    if (caractere == '\n') {
      if (bufferTCP.length() > 0) {
        processarLinhaTCP(bufferTCP);
        bufferTCP = "";
      }

      continue;
    }

    bufferTCP += caractere;

    // Guard against oversized invalid messages.
    if (bufferTCP.length() > 512) {
      Serial.println(
        "Buffer TCP excedeu 512 caracteres. A limpar."
      );

      bufferTCP = "";
    }
  }
}


// ======================================================
// HEARTBEAT
// ======================================================

void atualizarHeartbeat() {
  if (!clienteTCP.connected() || !registadoNoServidor) {
    return;
  }

  unsigned long agora = millis();

  if (agora - ultimoHeartbeat >= INTERVALO_HEARTBEAT) {
    ultimoHeartbeat = agora;

    enviarLinhaTCP(
      "PING|ID=" + String(VEHICLE_ID)
    );
  }

  if (
    ultimaMensagemServidor > 0
    && agora - ultimaMensagemServidor >= TIMEOUT_SERVIDOR
  ) {
    Serial.println(
      "Servidor sem resposta durante demasiado tempo. "
      "A forçar reconexão."
    );

    clienteTCP.stop();
    registadoNoServidor = false;
    enderecamentoObrigatorio = false;
    bufferTCP = "";

    pararMotores();
    estadoMovimento = PARADO;
  }
}


// ======================================================
// MOTOR SETUP
// ======================================================

void configurarPinosMotores() {
  pinMode(PINO_IN1, OUTPUT);
  pinMode(PINO_IN2, OUTPUT);

  pinMode(PINO_IN3, OUTPUT);
  pinMode(PINO_IN4, OUTPUT);

  ledcAttach(
    PINO_PWM_ESQUERDO,
    FREQUENCIA_PWM,
    RESOLUCAO_PWM
  );

  ledcAttach(
    PINO_PWM_DIREITO,
    FREQUENCIA_PWM,
    RESOLUCAO_PWM
  );

  pararMotores();
}


// ======================================================
// SETUP AND LOOP
// ======================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println(
    "========================================"
  );
  Serial.println(
    "Robot ESP32 - V2X multi-cliente"
  );
  Serial.println(
    "========================================"
  );

  configurarPinosMotores();

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);

  ultimaTentativaWiFi =
    millis() - INTERVALO_RECONEXAO_WIFI;

  ultimaTentativaTCP =
    millis() - INTERVALO_RECONEXAO_TCP;

  tentarLigarWiFi();
}


void loop() {
  verificarWiFi();
  verificarTCP();

  lerMensagensTCP();
  atualizarHeartbeat();
  atualizarMovimento();

  delay(5);
}