def preparar_mensagem_enderecada(vehicle_id, mensagem):
    """Ensure TO is present and convert legacy commands to addressed frames."""
    destino = normalizar_id_cliente(vehicle_id)
    mensagem = str(mensagem).rstrip("\n")

    if mensagem in MAPA_COMANDOS_ROBOT:
        return "COMMAND|FROM=INFRASTRUCTURE|TO={}|ACTION={}".format(
            destino, MAPA_COMANDOS_ROBOT[mensagem]
        )

    tipo, campos = parse_mensagem(mensagem)
    if not tipo:
        raise ValueError("Mensagem TCP vazia")
    endereco_existente = campos.get("TO")
    if endereco_existente is not None:
        if normalizar_id_cliente(endereco_existente) != destino:
            raise ValueError(
                "TO={} diferente do destino solicitado {}".format(
                    endereco_existente, destino
                )
            )
        return mensagem

    return mensagem + "|TO={}".format(destino)
