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
