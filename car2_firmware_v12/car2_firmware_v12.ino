#include <WiFi.h>

// ======================================================
// CAR 2 V2X IDENTIFICATION
// ======================================================

const char* VEHICLE_ID = "CAR_2";
const char* VEHICLE_TYPE = "CONNECTED_MANUAL";


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
String bufferSerial;


// ======================================================
// RECONNECTION
// ======================================================

const unsigned long INTERVALO_RECONEXAO_WIFI = 5000;
const unsigned long INTERVALO_RECONEXAO_TCP = 2000;

unsigned long ultimaTentativaWiFi = 0;
unsigned long ultimaTentativaTCP = 0;


// ======================================================
// CAR 2 LOCAL STATE
// ======================================================

String estadoCarro2 = "CONNECTED";
String localizacaoCarro2 = "UNKNOWN";


// ======================================================
// TCP TRANSMISSION
// ======================================================

void enviarLinhaTCP(const String& mensagem) {
  if (!clienteTCP.connected()) {
    Serial.println(
      "Não foi possível enviar: TCP desligado."
    );
    return;
  }

  clienteTCP.print(mensagem);
  clienteTCP.print('\n');

  Serial.print(">>> RPi: ");
  Serial.println(mensagem);
}


void enviarAck(
  const String& acao,
  const String& estado
) {
  String mensagem =
    "ACK|ID=" + String(VEHICLE_ID) +
    "|ACTION=" + acao +
    "|STATUS=" + estado;

  enviarLinhaTCP(mensagem);
}


void enviarEstado() {
  String mensagem =
    "STATUS|ID=" + String(VEHICLE_ID) +
    "|STATE=" + estadoCarro2 +
    "|LOCATION=" + localizacaoCarro2;

  enviarLinhaTCP(mensagem);
}


void enviarEstado(
  const String& estado,
  const String& localizacao
) {
  estadoCarro2 = estado;
  localizacaoCarro2 = localizacao;

  enviarEstado();
}


void enviarV2V(
  const String& destino,
  const String& evento
) {
  String mensagem =
    "V2V|TO=" + destino +
    "|EVENT=" + evento;

  enviarLinhaTCP(mensagem);
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

  bool ligadoAgora =
    WiFi.status() == WL_CONNECTED;

  if (ligadoAgora && !estavaLigado) {
    Serial.println("Wi-Fi ligado.");

    Serial.print("IP do ESP32 Carro 2: ");
    Serial.println(WiFi.localIP());
  }

  if (!ligadoAgora && estavaLigado) {
    Serial.println("Wi-Fi desligado.");

    clienteTCP.stop();
    registadoNoServidor = false;
    enderecamentoObrigatorio = false;
    bufferTCP = "";
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

  if (
    clienteTCP.connect(
      IP_RASPBERRY,
      PORTA_TCP
    )
  ) {
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
  }

  estavaLigado = ligadoAgora;

  if (!ligadoAgora) {
    tentarLigarTCP();
  }
}


// ======================================================
// FIELD PARSING
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


// ======================================================
// INFRASTRUCTURE EVENTS
// ======================================================

void processarEvento(
  const String& mensagem
) {
  String tipoEvento =
    obterCampo(mensagem, "TYPE");

  Serial.print(
    "Evento da infraestrutura: "
  );
  Serial.println(tipoEvento);


  if (tipoEvento == "RFID_SAFE_PLACE_REQUIRED") {
    Serial.println();
    Serial.println(
      "Existe um SAIR1 pendente e o Carro 2 esta a bloquear a entrada."
    );
    Serial.println(
      "Apresente primeiro o cartao RFID do Carro 2."
    );
    Serial.println(
      "So depois mova manualmente o Carro 2 para o SAFE PLACE."
    );

    enviarEstado(
      "WAITING_RFID_BEFORE_SAFE_PLACE",
      "ENTRANCE"
    );

    enviarAck(
      "RFID_SAFE_PLACE_REQUIRED",
      "RECEIVED"
    );
    return;
  }

  if (tipoEvento == "WAIT_FOR_ROBOT_RFID") {
    Serial.println();
    Serial.println(
      "SAIR2 pendente: o ROBOT esta a bloquear a entrada."
    );
    Serial.println(
      "Apresente primeiro o chip RFID do ROBOT."
    );
    Serial.println(
      "Depois a infraestrutura envia automaticamente o ROBOT ao SAFE PLACE."
    );

    enviarEstado(
      "WAITING_ROBOT_RFID",
      "PLACE_2"
    );

    enviarAck(
      "WAIT_FOR_ROBOT_RFID",
      "RECEIVED"
    );
    return;
  }

  if (tipoEvento == "RETURN_TO_ENTRANCE") {
    Serial.println();
    Serial.println(
      "Saida do ROBOT concluida e entrada livre."
    );
    Serial.println(
      "Mova agora manualmente o Carro 2 do SAFE PLACE para a ENTRADA."
    );

    enviarEstado(
      "RETURNING_MANUAL",
      "FROM_SAFE_PLACE_TO_ENTRANCE"
    );

    enviarAck(
      "RETURN_TO_ENTRANCE",
      "RECEIVED"
    );
    return;
  }

  if (tipoEvento == "RETURN_TO_ENTRANCE_CONFIRMED") {
    Serial.println();
    Serial.println(
      "Carro 2 detetado novamente na entrada."
    );
    Serial.println(
      "Apresente o cartao RFID para efetuar a entrada."
    );

    enviarEstado(
      "WAITING_RFID_ENTRY",
      "ENTRANCE"
    );

    enviarAck(
      "RETURN_TO_ENTRANCE_CONFIRMED",
      "RECEIVED"
    );
    return;
  }

  if (tipoEvento == "WAIT_FOR_ROBOT_SAFE_PLACE") {
    Serial.println();
    Serial.println(
      "SAIR2 pedido, mas o ROBOT esta a bloquear a entrada."
    );
    Serial.println(
      "Aguarde: a infraestrutura esta a enviar o ROBOT para o SAFE PLACE."
    );
    Serial.println(
      "Nao mova ainda o Carro 2 do Lugar 2."
    );

    enviarEstado(
      "WAITING_CLEAR_PATH",
      "PLACE_2"
    );

    enviarAck(
      "WAIT_FOR_ROBOT_SAFE_PLACE",
      "RECEIVED"
    );
    return;
  }

  if (tipoEvento == "ROBOT_SAFE_PLACE_CONFIRMED") {
    Serial.println();
    Serial.println(
      "ROBOT no safe place e entrada livre."
    );
    Serial.println(
      "A infraestrutura vai agora autorizar a saida manual do Carro 2."
    );

    enviarEstado(
      "WAITING_MANUAL_EXIT",
      "PLACE_2"
    );

    enviarAck(
      "ROBOT_SAFE_PLACE_CONFIRMED",
      "RECEIVED"
    );
    return;
  }

  if (tipoEvento == "MANUAL_EXIT_TO_CORNER") {
    Serial.println();
    Serial.println(
      "SAIR2 AUTORIZADO: mova manualmente o Carro 2 "
      "do Lugar 2 ate ao CANTO."
    );

    enviarEstado(
      "EXITING_MANUAL",
      "FROM_PLACE_2_TO_CORNER"
    );

    enviarAck(
      "MANUAL_EXIT_TO_CORNER",
      "RECEIVED"
    );
    return;
  }

  if (tipoEvento == "MANUAL_EXIT_TO_ENTRANCE") {
    Serial.println();
    Serial.println(
      "Carro 2 detetado no canto. "
      "Mova-o agora ate a ENTRADA/SAIDA."
    );

    enviarEstado(
      "EXITING_MANUAL",
      "FROM_CORNER_TO_ENTRANCE"
    );

    enviarAck(
      "MANUAL_EXIT_TO_ENTRANCE",
      "RECEIVED"
    );
    return;
  }

  if (tipoEvento == "RFID_EXIT_REQUIRED") {
    Serial.println();
    Serial.println(
      "Carro 2 detetado na entrada. "
      "Apresente o cartao RFID para confirmar a saida."
    );

    enviarEstado(
      "WAITING_RFID_EXIT",
      "ENTRANCE"
    );

    enviarAck(
      "RFID_EXIT_REQUIRED",
      "RECEIVED"
    );
    return;
  }

  if (tipoEvento == "MOVE_TO_SAFE_PLACE") {
    Serial.println();
    Serial.println(
      "ATENCAO: Mova o Carro 2 ate ao SAFE PLACE."
    );
    Serial.println(
      "O robot esta no canto e pretende sair."
    );

    enviarEstado(
      "MOVING_TO_SAFE_PLACE",
      "ENTRANCE"
    );

    enviarAck(
      "MOVE_TO_SAFE_PLACE",
      "RECEIVED"
    );
    return;
  }

  if (tipoEvento == "SAFE_PLACE_CONFIRMED") {
    Serial.println(
      "Entrada libertada. O robot pode continuar a saida."
    );

    enviarEstado(
      "WAITING",
      "SAFE_PLACE"
    );

    enviarAck(
      "SAFE_PLACE_CONFIRMED",
      "RECEIVED"
    );
    return;
  }

  if (tipoEvento == "WAIT_FOR_ROBOT_PRIORITY") {
    Serial.println(
      "Os dois veiculos pretendem sair. "
      "Aguarde: o ROBOT tem prioridade."
    );

    enviarEstado(
      "WAITING_ROBOT_EXIT",
      localizacaoCarro2
    );

    enviarAck(
      "WAIT_FOR_ROBOT_PRIORITY",
      "RECEIVED"
    );
    return;
  }

  if (tipoEvento == "SYSTEM_STATE") {
    String estadoSistema =
      obterCampo(mensagem, "STATE");

    Serial.print("Estado do parque: ");
    Serial.println(estadoSistema);
    return;
  }

  if (tipoEvento == "ENTRY_AUTHORIZED") {
    String lugar =
      obterCampo(mensagem, "PLACE");

    Serial.print(
      "Entrada autorizada para: "
    );
    Serial.println(lugar);

    enviarEstado(
      "PARKING",
      "ENTRANCE"
    );

    enviarAck(
      "ENTRY_AUTHORIZED",
      "RECEIVED"
    );
    return;
  }

  if (tipoEvento == "PARKING_COMPLETED") {
    String lugar =
      obterCampo(mensagem, "PLACE");

    Serial.print(
      "Estacionamento confirmado em: "
    );
    Serial.println(lugar);

    enviarEstado(
      "PARKED",
      "PLACE_2"
    );

    enviarAck(
      "PARKING_COMPLETED",
      "RECEIVED"
    );
    return;
  }

  if (tipoEvento == "EXIT_STARTED") {
    enviarEstado(
      "EXITING",
      "PLACE_2"
    );

    enviarAck(
      "EXIT_STARTED",
      "RECEIVED"
    );
    return;
  }

  if (tipoEvento == "EXIT_CONTINUE") {
    Serial.println(
      "A infraestrutura autorizou "
      "a continuação da saída."
    );

    enviarEstado(
      "EXITING",
      "CORNER"
    );

    enviarAck(
      "EXIT_CONTINUE",
      "RECEIVED"
    );
    return;
  }

  if (tipoEvento == "EXIT_CONFIRMED") {
    enviarEstado(
      "OUTSIDE",
      "OUTSIDE"
    );

    enviarAck(
      "EXIT_CONFIRMED",
      "RECEIVED"
    );
    return;
  }

  if (tipoEvento == "ENTRANCE_AVAILABLE") {
    Serial.println(
      "A entrada do parque está livre."
    );

    enviarAck(
      "ENTRANCE_AVAILABLE",
      "RECEIVED"
    );
    return;
  }

  if (tipoEvento == "STATE_CHANGED") {
    String estadoSistema =
      obterCampo(mensagem, "STATE");

    Serial.print(
      "Novo estado da infraestrutura: "
    );
    Serial.println(estadoSistema);
    return;
  }
}


// ======================================================
// INFRASTRUCTURE-MEDIATED V2V COMMUNICATION
// ======================================================

void processarV2V(
  const String& mensagem
) {
  String origem =
    obterCampo(mensagem, "FROM");

  String evento =
    obterCampo(mensagem, "EVENT");

  Serial.print(
    "V2V via infraestrutura | origem="
  );
  Serial.print(origem);
  Serial.print(" | evento=");
  Serial.println(evento);

  if (evento == "ROBOT_ENTERING") {
    Serial.println(
      "Informação recebida: "
      "o robot está a entrar."
    );
  } else if (evento == "ROBOT_PARKED") {
    Serial.println(
      "Informação recebida: "
      "o robot estacionou."
    );
  } else if (evento == "ROBOT_EXITING") {
    Serial.println(
      "Informação recebida: "
      "o robot iniciou a saída."
    );
  } else if (evento == "ROBOT_EXITED") {
    Serial.println(
      "Informação recebida: "
      "o robot saiu do parque."
    );
  } else if (evento == "VEHICLE_CONNECTED") {
    Serial.println(
      "Outro veículo ligou-se "
      "à infraestrutura."
    );
  } else if (
    evento == "VEHICLE_DISCONNECTED"
  ) {
    Serial.println(
      "Outro veículo desligou-se "
      "da infraestrutura."
    );
  }

  enviarAck(
    "V2V_" + evento,
    "RECEIVED"
  );
}


// ======================================================
// TCP MESSAGE PROCESSING
// ======================================================

void processarLinhaTCP(
  String mensagem
) {
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

  if (
    mensagem.startsWith("WELCOME|")
  ) {
    String idRecebido =
      obterCampo(mensagem, "ID");

    if (idRecebido == VEHICLE_ID) {
      enderecamentoObrigatorio = (obterCampo(mensagem, "PROTOCOL") == "2");
      registadoNoServidor = true;

      Serial.println(
        "Carro 2 registado no "
        "servidor multi-cliente."
      );

      enviarEstado(
        "CONNECTED",
        "UNKNOWN"
      );
    }

    return;
  }

  if (
    mensagem.startsWith("PONG|")
  ) {
    return;
  }

  if (
    mensagem.startsWith("EVENT|")
  ) {
    processarEvento(mensagem);
    return;
  }

  if (
    mensagem.startsWith("V2V|")
  ) {
    processarV2V(mensagem);
    return;
  }

  if (
    mensagem.startsWith("ERROR|")
  ) {
    Serial.print(
      "Erro comunicado pelo servidor: "
    );
    Serial.println(mensagem);
    return;
  }

  if (
    mensagem.startsWith("COMMAND|")
  ) {
    String acao =
      obterCampo(mensagem, "ACTION");

    Serial.print(
      "Comando recebido para o Carro 2: "
    );
    Serial.println(acao);

    enviarAck(
      acao,
      "RECEIVED"
    );

    return;
  }

  Serial.print(
    "Mensagem não reconhecida: "
  );
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

    if (bufferTCP.length() > 512) {
      Serial.println(
        "Buffer TCP excedeu 512 "
        "caracteres. A limpar."
      );

      bufferTCP = "";
    }
  }
}


// ======================================================
// HEARTBEAT
// ======================================================

void atualizarHeartbeat() {
  if (
    !clienteTCP.connected()
    || !registadoNoServidor
  ) {
    return;
  }

  unsigned long agora = millis();

  if (
    agora - ultimoHeartbeat
    >= INTERVALO_HEARTBEAT
  ) {
    ultimoHeartbeat = agora;

    enviarLinhaTCP(
      "PING|ID=" + String(VEHICLE_ID)
    );
  }

  if (
    ultimaMensagemServidor > 0
    && agora - ultimaMensagemServidor
       >= TIMEOUT_SERVIDOR
  ) {
    Serial.println(
      "Servidor sem resposta durante "
      "demasiado tempo. "
      "A forçar reconexão."
    );

    clienteTCP.stop();
    registadoNoServidor = false;
    enderecamentoObrigatorio = false;
    bufferTCP = "";
  }
}


// ======================================================
// SERIAL MONITOR TEST COMMANDS
// ======================================================

void mostrarAjudaSerial() {
  Serial.println();
  Serial.println(
    "Comandos de teste disponíveis:"
  );
  Serial.println(
    "  STATUS"
  );
  Serial.println(
    "  PARKING"
  );
  Serial.println(
    "  PARKED"
  );
  Serial.println(
    "  EXITING"
  );
  Serial.println(
    "  OUTSIDE"
  );
  Serial.println(
    "  V2V ROBOT TEST"
  );
  Serial.println(
    "  V2V ROBOT EXIT_REQUEST"
  );
  Serial.println(
    "  HELP"
  );
  Serial.println();
}


void processarComandoSerial(
  String comando
) {
  comando.trim();

  if (comando.length() == 0) {
    return;
  }

  comando.toUpperCase();

  if (comando == "HELP") {
    mostrarAjudaSerial();
    return;
  }

  if (comando == "STATUS") {
    enviarEstado();
    return;
  }

  if (comando == "PARKING") {
    enviarEstado(
      "PARKING",
      "ENTRANCE"
    );
    return;
  }

  if (comando == "PARKED") {
    enviarEstado(
      "PARKED",
      "PLACE_2"
    );
    return;
  }

  if (comando == "EXITING") {
    enviarEstado(
      "EXITING",
      "PLACE_2"
    );
    return;
  }

  if (comando == "OUTSIDE") {
    enviarEstado(
      "OUTSIDE",
      "OUTSIDE"
    );
    return;
  }

  if (
    comando.startsWith("V2V ")
  ) {
    int primeiroEspaco =
      comando.indexOf(' ');

    int segundoEspaco =
      comando.indexOf(
        ' ',
        primeiroEspaco + 1
      );

    if (segundoEspaco < 0) {
      Serial.println(
        "Formato inválido. "
        "Use: V2V ROBOT EVENTO"
      );
      return;
    }

    String destino =
      comando.substring(
        primeiroEspaco + 1,
        segundoEspaco
      );

    String evento =
      comando.substring(
        segundoEspaco + 1
      );

    enviarV2V(
      destino,
      evento
    );
    return;
  }

  Serial.print(
    "Comando série desconhecido: "
  );
  Serial.println(comando);
}


void lerMonitorSerial() {
  while (Serial.available()) {
    char caractere = Serial.read();

    if (caractere == '\r') {
      continue;
    }

    if (caractere == '\n') {
      if (bufferSerial.length() > 0) {
        processarComandoSerial(
          bufferSerial
        );

        bufferSerial = "";
      }

      continue;
    }

    bufferSerial += caractere;

    if (bufferSerial.length() > 128) {
      bufferSerial = "";
    }
  }
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
    "ESP32 Carro 2 - Cliente V2X"
  );
  Serial.println(
    "========================================"
  );

  Serial.println(
    "Este firmware não controla motores "
    "nem sensores."
  );

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);

  ultimaTentativaWiFi =
    millis() - INTERVALO_RECONEXAO_WIFI;

  ultimaTentativaTCP =
    millis() - INTERVALO_RECONEXAO_TCP;

  mostrarAjudaSerial();
  tentarLigarWiFi();
}


void loop() {
  verificarWiFi();
  verificarTCP();

  lerMensagensTCP();
  atualizarHeartbeat();
  lerMonitorSerial();

  delay(5);
}