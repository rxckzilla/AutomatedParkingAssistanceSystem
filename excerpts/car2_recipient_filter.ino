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
