#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import datetime
import json
import os
import queue
import shutil
import socket
import threading
import time

import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
FICHEIRO_ESTADO = os.path.join(BASE_DIR, "parking_state.json")
FICHEIRO_ESTADO_BACKUP = os.path.join(BASE_DIR, "parking_state.backup.json")

# Optional JSON Lines history. Each line is an independent JSON object.
# This file is used only for auditing/debugging and NEVER determines
# whether a vehicle may enter or exit. A damaged log therefore does not block the parking system.
FICHEIRO_EVENTOS = os.path.join(BASE_DIR, "parking_events.jsonl")

# Actual UIDs from the supplied parking_state.json file.
# Used only when creating or repairing a new state; if the existing JSON already contains
# valid UIDs, those values are preserved.
UID_ROBOT_PADRAO = 583236613027
UID_CARRO_2_PADRAO = 40776534406
estado_persistente = {}
estado_lock = threading.RLock()

ID_ROBOT = "ROBOT"
ID_CARRO_2 = "CAR_2"
TIPO_ROBOT = "AUTONOMOUS"
TIPO_CARRO_2 = "CONNECTED_MANUAL"
IDS_PERMITIDOS = {ID_ROBOT: TIPO_ROBOT, ID_CARRO_2: TIPO_CARRO_2}

TRIG_ENTRADA = 23
ECHO_ENTRADA = 24
TRIG_CANTO = 5
ECHO_CANTO = 6
TRIG_LUGAR_1 = 17
ECHO_LUGAR_1 = 27
TRIG_LUGAR_2 = 22
ECHO_LUGAR_2 = 26

HOST = "0.0.0.0"
PORTA_TCP = 1234
MAX_CLIENTES_PENDENTES = 10
TIMEOUT_HELLO = 10.0
TIMEOUT_CLIENTE = 1.0
TIMEOUT_HEARTBEAT = 20.0
INTERVALO_VERIFICACAO_CLIENTES = 2.0
servidor_tcp = None
clientes = {}
clientes_lock = threading.RLock()

LIMITE_ENTRADA_CM = 15.0

# During a commanded SAIR1 operation, the entrance is considered
# reached at 20 cm.
LIMITE_ENTRADA_SAIR1_CM = 20.0

# The ROBOT's automatic/legacy exit retains the previous 20 cm threshold.
LIMITE_ENTRADA_SAIDA_ROBOT_CM = 20.0

LIMITE_CANTO_ENTRADA_CM = 16.0

# The ROBOT's automatic/legacy exit retains the previous value.
LIMITE_CANTO_SAIDA_ROBOT_CM = 25.0

# Only for SAIR1, the Robot must approach closer to the corner.
LIMITE_CANTO_SAIR1_CM = 20.0

# Retain the previous value for Car 2's manual exit
# and for other scenarios that have already been calibrated.
LIMITE_CANTO_SAIDA_CM = 15.0

# General space 1 threshold used for parking validation and state tracking.
LIMITE_LUGAR_1_CM = 20.0

# Only during the ROBOT's automatic parking maneuver, the stop
# in space 1 is triggered at 15 cm.
LIMITE_LUGAR_1_ESTACIONAMENTO_CM = 15.0

LIMITE_LUGAR_2_CM = 10.0  # OCCUPIED: <= 10 cm (unchanged)
# Space 2 is next to the wall: FREE requires a valid reading >= 15 cm.
# Readings from 10 to <15 cm form an unconfirmed transition zone.
LIMITE_LUGAR_2_LIVRE_CM = 15.0

# SLOT MANAGEMENT (logical position inferred from the existing sensors)
# SLOT 0: starting area -> entrance
# SLOT 1: entrance -> corner
# SLOT 2: corner -> space 1 sensor
# SLOT 3: space 1 sensor -> space 2
# To avoid interpreting an isolated fluctuation as passage, CAR_2
# requires three normal confirmations at the CORNER and SPACE 1; to authorize
# the ROBOT's entry, both sensors must also indicate that the route is clear.
CONFIRMACOES_SLOT = 3
TIMEOUT_ECHO = 0.03

# Faster overall polling cycle.
# Previously: 60 ms between sensors + 50 ms at the end of the loop.
# Now: 30 ms between sensors + 20 ms at the end of the loop.
# This substantially reduces the interval in which a vehicle can pass
# a sensor without being sampled.
PAUSA_ENTRE_SENSORES = 0.03
PAUSA_LOOP = 0.02
INTERVALO_RESUMO = 0.5
INTERVALO_MENSAGEM = 2.0
PAUSA_ANTES_COMANDO_CURVA = 0.30
TEMPO_MAXIMO_ATE_CANTO = 25.0
TEMPO_MAXIMO_ATE_LUGAR_1 = 25.0
TEMPO_MAXIMO_SAIDA_ATE_CANTO = 30.0
TEMPO_MAXIMO_SAIDA_ATE_ENTRADA = 30.0

# Safety checks for the new START scenario.
TEMPO_MAXIMO_START_ATE_ENTRADA = 30.0
CONFIRMACOES_SENSOR = 3

# Final ROBOT stop in space 1:
# use two confirmations to reduce the delay before the STOP command.
# The remaining sensors and scenarios still use three confirmations.
CONFIRMACOES_PARAGEM_LUGAR_1 = 2

# Fast confirmation ONLY during automatic parking in space 1.
# The first reading comes from the normal cycle; if it is <= 20 cm, the Raspberry Pi
# waits only 25 ms and measures the space 1 sensor again.
PAUSA_CONFIRMACAO_RAPIDA_LUGAR_1 = 0.025

# Fast confirmation ONLY when reaching the corner sensor during
# automatic parking.
PAUSA_CONFIRMACAO_RAPIDA_CANTO = 0.025

# Fast confirmation ONLY when the ROBOT reaches the entrance
# during the SAIR1 command.
PAUSA_CONFIRMACAO_RAPIDA_ENTRADA_SAIR1 = 0.025

# Fast confirmation ONLY when the ROBOT returns from the SAFE PLACE
# and approaches the entrance sensor.
PAUSA_CONFIRMACAO_RAPIDA_ENTRADA_RETORNO = 0.025

# New START scenario:
# the first reading comes from the normal cycle; the second is taken 25 ms later.
PAUSA_CONFIRMACAO_RAPIDA_ENTRADA_START = 0.025

# Entrance conflict while the ROBOT exits: use the current cycle's first sample
# and TWO additional readings at 25 ms intervals, all <= 15 cm.
PAUSA_CONFIRMACAO_TRIPLA_ENTRADA_CONFLITO = 0.025

# While the ROBOT travels from Corner -> Entrance during SAIR1, do not wait for
# the complete four-sensor cycle. Sample the entrance approximately
# every 50 ms. Retain the final 25 ms confirmation.
PAUSA_POLLING_ENTRADA_SAIR1 = 0.050


# ROBOT stop at the entrance during SAIR1:
# use two confirmations to reduce the delay, as in space 1.
CONFIRMACOES_PARAGEM_ENTRADA_SAIR1 = 2

# Corner turn during SAIR1: two consecutive readings <= 25 cm.
CONFIRMACOES_CANTO_SAIR1 = 2

AGUARDAR_VEICULO_ENTRADA = "AGUARDAR_VEICULO_ENTRADA"

# New scenario:
# the ROBOT starts outside the parking area and, after the "start" command,
# moves automatically toward the entrance sensor.
ROBOT_START_ATE_ENTRADA = "ROBOT_START_ATE_ENTRADA"

# START requested while a vehicle occupies the entrance:
# first, CAR_2 must be identified by RFID;
# authorize its parking maneuver and only then, once the entrance
# is physically clear, automatically resume the ROBOT's START operation.
AGUARDAR_RFID_CARRO_2_PARA_START = "AGUARDAR_RFID_CARRO_2_PARA_START"
AGUARDAR_ENTRADA_LIVRE_PARA_START_APOS_CARRO_2 = (
    "AGUARDAR_ENTRADA_LIVRE_PARA_START_APOS_CARRO_2"
)

AGUARDAR_RFID_ENTRADA = "AGUARDAR_RFID_ENTRADA"
CARRO_2_A_ESTACIONAR = "CARRO_2_A_ESTACIONAR"

# Automatic detection of CAR_2's exit while the ROBOT waits for RFID at the entrance.
# The exit remains suspended until the ROBOT reaches the safe spot.
AGUARDAR_ROBOT_NO_SAFE_PLACE_PARA_SAIDA_AUTO_CARRO_2 = (
    "AGUARDAR_ROBOT_NO_SAFE_PLACE_PARA_SAIDA_AUTO_CARRO_2"
)
ROBOT_AVANCAR_ATE_CANTO = "ROBOT_AVANCAR_ATE_CANTO"
ROBOT_AVANCAR_ATE_LUGAR_1 = "ROBOT_AVANCAR_ATE_LUGAR_1"
ESTACIONADO_LUGAR_1 = "ESTACIONADO_LUGAR_1"
ESTACIONADO_LUGAR_2 = "ESTACIONADO_LUGAR_2"
ROBOT_A_SAIR_ATE_ENTRADA = "ROBOT_A_SAIR_ATE_ENTRADA"
ROBOT_SAIDA_COMANDO_ATE_CANTO = "ROBOT_SAIDA_COMANDO_ATE_CANTO"
ROBOT_SAIDA_COMANDO_ATE_ENTRADA = "ROBOT_SAIDA_COMANDO_ATE_ENTRADA"
CARRO_2_SAIDA_COMANDO_ATE_CANTO = "CARRO_2_SAIDA_COMANDO_ATE_CANTO"
CARRO_2_SAIDA_COMANDO_ATE_ENTRADA = "CARRO_2_SAIDA_COMANDO_ATE_ENTRADA"
AGUARDAR_RFID_BLOQUEADOR_PARA_SAIR1 = "AGUARDAR_RFID_BLOQUEADOR_PARA_SAIR1"
AGUARDAR_RFID_BLOQUEADOR_PARA_SAIR2 = "AGUARDAR_RFID_BLOQUEADOR_PARA_SAIR2"
AGUARDAR_ENTRADA_LIVRE_PARA_SAIR1 = "AGUARDAR_ENTRADA_LIVRE_PARA_SAIR1"
AGUARDAR_ENTRADA_LIVRE_PARA_SAIR2 = "AGUARDAR_ENTRADA_LIVRE_PARA_SAIR2"
AGUARDAR_CARRO_2_NO_SAFE_PLACE = "AGUARDAR_CARRO_2_NO_SAFE_PLACE"
ROBOT_A_SAIR_APOS_CANTO = "ROBOT_A_SAIR_APOS_CANTO"
AGUARDAR_RFID_SAIDA_ROBOT = "AGUARDAR_RFID_SAIDA_ROBOT"
# Car 2 reaches the entrance after SAIR1 (or a manual exit) has started.
AGUARDAR_RFID_CARRO_2_DURANTE_SAIDA_ROBOT = (
    "AGUARDAR_RFID_CARRO_2_DURANTE_SAIDA_ROBOT"
)
AGUARDAR_ENTRADA_LIVRE_PARA_RETOMAR_SAIDA_ROBOT = (
    "AGUARDAR_ENTRADA_LIVRE_PARA_RETOMAR_SAIDA_ROBOT"
)
AGUARDAR_ACK_RETOMA_SAIDA_ROBOT = "AGUARDAR_ACK_RETOMA_SAIDA_ROBOT"
CARRO_2_A_SAIR_ATE_CANTO = "CARRO_2_A_SAIR_ATE_CANTO"
AGUARDAR_RFID_SAIDA_CARRO_2 = "AGUARDAR_RFID_SAIDA_CARRO_2"
AGUARDAR_ENTRADA_LIVRE_APOS_SAIDA = "AGUARDAR_ENTRADA_LIVRE_APOS_SAIDA"
AGUARDAR_ENTRADA_LIVRE_PARA_RETORNO_ROBOT = "AGUARDAR_ENTRADA_LIVRE_PARA_RETORNO_ROBOT"
AGUARDAR_ROBOT_RETORNAR_ENTRADA = "AGUARDAR_ROBOT_RETORNAR_ENTRADA"
AGUARDAR_ENTRADA_LIVRE_PARA_RETORNO_CARRO_2 = "AGUARDAR_ENTRADA_LIVRE_PARA_RETORNO_CARRO_2"
AGUARDAR_CARRO_2_RETORNAR_ENTRADA = "AGUARDAR_CARRO_2_RETORNAR_ENTRADA"
ERRO_PERCURSO = "ERRO_PERCURSO"

estado_sistema = AGUARDAR_VEICULO_ENTRADA
instante_inicio_estado = time.monotonic()
programa_ativo = True
fila_rfid = queue.Queue()
fila_comandos_interface = queue.Queue()
ultimo_resumo = 0.0
ultima_mensagem = 0.0
ultima_verificacao_clientes = 0.0
contador_lugar_1_ocupado = 0
contador_lugar_1_livre = 0
contador_lugar_2_ocupado = 0
contador_lugar_2_livre = 0
contador_canto_saida = 0
contador_entrada = 0
contador_entrada_livre = 0

# Independent of the counters for the ROBOT's route and SAIR2.
contador_canto_slot_2_ocupado = 0
contador_canto_slot_2_livre = 0

# Independent counter for the ROBOT's arrival at the entrance during an exit.
# Uses LIMITE_ENTRADA_SAIDA_ROBOT_CM (20 cm), without changing
# normal entrance behavior for Car 2 or new arrivals.
contador_entrada_saida_robot = 0

# Dedicated counter for the ROBOT's arrival at the entrance during SAIR1.
# Uses 25 cm and two confirmations, without changing the automatic/legacy exit.
contador_entrada_sair1 = 0

sensores_inicializados = False

# Temporary context for the ROBOT's suspension.
# Freeze phase transitions while CAR_2 is identified
# and moved manually to the safe spot.
saida_robot_estado_interrompido = None
saida_robot_pausa_confirmada = False
saida_robot_retomada_confirmada = False
robot_avanco_final_confirmado = False



def agora_iso():
    return datetime.datetime.now().replace(microsecond=0).isoformat()


def registar_evento_historico(evento, veiculo=None, detalhes=None):
    """
    Record an event in parking_events.jsonl.

    Important:
    - this is history/auditing only;
    - parking control does NOT depend on this file;
    - if writing fails, the system continues operating.
    """
    registo = {
        "timestamp": agora_iso(),
        "event": str(evento),
        "vehicle": veiculo,
        "details": detalhes or {}
    }

    try:
        with open(FICHEIRO_EVENTOS, "a") as ficheiro:
            json.dump(registo, ficheiro, sort_keys=True)
            ficheiro.write("\n")
            ficheiro.flush()
            os.fsync(ficheiro.fileno())
    except (OSError, IOError) as erro:
        print(
            "Aviso: não foi possível escrever parking_events.jsonl: {}".format(
                erro
            )
        )


def normalizar_id_cliente(vehicle_id):
    if vehicle_id is None:
        return None
    valor = str(vehicle_id).strip().upper()
    aliases = {
        "ROBOT": ID_ROBOT,
        "ROBO": ID_ROBOT,
        "CAR_2": ID_CARRO_2,
        "CAR2": ID_CARRO_2,
        "CARRO_2": ID_CARRO_2,
        "CARRO2": ID_CARRO_2
    }
    return aliases.get(valor, valor)


def parse_mensagem(mensagem):
    partes = [p.strip() for p in mensagem.strip().split("|")]
    if not partes or not partes[0]:
        return "", {}
    tipo = partes[0].upper()
    campos = {}
    for parte in partes[1:]:
        if "=" in parte:
            chave, valor = parte.split("=", 1)
            campos[chave.strip().upper()] = valor.strip()
    return tipo, campos


def criar_estado_padrao():
    return {
        "version": 3,
        "updated_at": agora_iso(),
        "last_rfid": {
            "uid": None,
            "vehicle": None,
            "action": None,
            "timestamp": None
        },
        "vehicles": {
            "robot": {
                "uid": UID_ROBOT_PADRAO,
                "inside": False,
                "location": "outside",
                "network_id": ID_ROBOT,
                "type": TIPO_ROBOT,
                "connected": False,
                "last_seen": None,
                "slot": None
            },
            "car_2": {
                "uid": UID_CARRO_2_PADRAO,
                "inside": False,
                "location": "outside",
                "network_id": ID_CARRO_2,
                "type": TIPO_CARRO_2,
                "connected": False,
                "last_seen": None,
                "slot": None,
                "place_1_passage_detected": False
            }
        },
        "parking": {
            "place_1": {"occupied": False, "vehicle": None},
            "place_2": {"occupied": False, "vehicle": None, "reserved_by": None}
        },
        "traffic": {
            "blocked": False,
            "reason": None
        }
    }


def garantir_campos_json():
    """
    Repair missing fields without destroying existing valid information.

    This allows direct use of the existing parking_state.json v3 file.
    """
    global estado_persistente

    if not isinstance(estado_persistente, dict):
        estado_persistente = {}

    padrao = criar_estado_padrao()

    estado_persistente["version"] = 3

    if not isinstance(estado_persistente.get("last_rfid"), dict):
        estado_persistente["last_rfid"] = {}
    for campo, valor in padrao["last_rfid"].items():
        if campo not in estado_persistente["last_rfid"]:
            estado_persistente["last_rfid"][campo] = valor

    if not isinstance(estado_persistente.get("vehicles"), dict):
        estado_persistente["vehicles"] = {}

    for veiculo in ("robot", "car_2"):
        if not isinstance(
            estado_persistente["vehicles"].get(veiculo),
            dict
        ):
            estado_persistente["vehicles"][veiculo] = {}

        for campo, valor in padrao["vehicles"][veiculo].items():
            if campo not in estado_persistente["vehicles"][veiculo]:
                estado_persistente["vehicles"][veiculo][campo] = valor

    if not isinstance(estado_persistente.get("parking"), dict):
        estado_persistente["parking"] = {}

    for lugar in ("place_1", "place_2"):
        if not isinstance(
            estado_persistente["parking"].get(lugar),
            dict
        ):
            estado_persistente["parking"][lugar] = {}

        for campo, valor in padrao["parking"][lugar].items():
            if campo not in estado_persistente["parking"][lugar]:
                estado_persistente["parking"][lugar][campo] = valor

    if not isinstance(estado_persistente.get("traffic"), dict):
        estado_persistente["traffic"] = {}

    for campo, valor in padrao["traffic"].items():
        if campo not in estado_persistente["traffic"]:
            estado_persistente["traffic"][campo] = valor

    if "updated_at" not in estado_persistente:
        estado_persistente["updated_at"] = agora_iso()


def guardar_estado():
    """
    Write parking_state.json atomically.

    1. Create/update a backup of the last valid JSON file.
    2. Write to .tmp.
    3. Flush the data to disk.
    4. Atomically replace the primary file.
    """
    with estado_lock:
        estado_persistente["updated_at"] = agora_iso()

        temporario = FICHEIRO_ESTADO + ".tmp"

        try:
            if os.path.exists(FICHEIRO_ESTADO):
                # Replace the backup only if the current primary file
                # is still valid JSON. A corrupted parking_state.json
                # therefore never overwrites the last valid backup.
                try:
                    carregar_json_de_ficheiro(FICHEIRO_ESTADO)
                    principal_valido = True
                except (ValueError, OSError, TypeError):
                    principal_valido = False

                if principal_valido:
                    shutil.copy2(
                        FICHEIRO_ESTADO,
                        FICHEIRO_ESTADO_BACKUP
                    )
        except OSError as erro:
            print(
                "Aviso: não foi possível atualizar o backup do estado: {}".format(
                    erro
                )
            )

        with open(temporario, "w") as ficheiro:
            json.dump(
                estado_persistente,
                ficheiro,
                indent=4,
                sort_keys=True
            )
            ficheiro.flush()
            os.fsync(ficheiro.fileno())

        os.replace(temporario, FICHEIRO_ESTADO)


def carregar_json_de_ficheiro(caminho):
    with open(caminho, "r") as ficheiro:
        dados = json.load(ficheiro)

    if not isinstance(dados, dict):
        raise ValueError("A raiz do JSON não é um objeto.")

    return dados


def carregar_estado():
    global estado_persistente

    print("JSON de estado utilizado pelo programa:")
    print(FICHEIRO_ESTADO)
    print("Histórico opcional:")
    print(FICHEIRO_EVENTOS)

    if not os.path.exists(FICHEIRO_ESTADO):
        estado_persistente = criar_estado_padrao()
        guardar_estado()
        registar_evento_historico(
            "STATE_FILE_CREATED",
            detalhes={"file": "parking_state.json"}
        )
        print("Foi criado um novo parking_state.json.")
        return

    try:
        estado_persistente = carregar_json_de_ficheiro(
            FICHEIRO_ESTADO
        )
        garantir_campos_json()

        # A TCP connection is always runtime state and must not be
        # restored from a previous execution.
        estado_persistente["vehicles"]["robot"]["connected"] = False
        estado_persistente["vehicles"]["car_2"]["connected"] = False

        guardar_estado()
        print("Estado JSON carregado e validado com sucesso.")
        return

    except (ValueError, OSError, TypeError) as erro:
        print(
            "parking_state.json inválido ou ilegível: {}".format(erro)
        )

    # Attempt recovery from the backup first.
    if os.path.exists(FICHEIRO_ESTADO_BACKUP):
        try:
            estado_persistente = carregar_json_de_ficheiro(
                FICHEIRO_ESTADO_BACKUP
            )
            garantir_campos_json()

            estado_persistente["vehicles"]["robot"]["connected"] = False
            estado_persistente["vehicles"]["car_2"]["connected"] = False

            guardar_estado()
            registar_evento_historico(
                "STATE_RECOVERED_FROM_BACKUP"
            )
            print(
                "Estado recuperado de parking_state.backup.json."
            )
            return
        except (ValueError, OSError, TypeError) as erro_backup:
            print(
                "Backup também inválido: {}".format(erro_backup)
            )

    # If both primary and backup files fail, preserve the damaged primary file and
    # create a clean state. Reconciliation with the sensors occurs afterward.
    if os.path.exists(FICHEIRO_ESTADO):
        nome_corrompido = (
            FICHEIRO_ESTADO
            + ".erro_"
            + datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        )
        try:
            os.replace(FICHEIRO_ESTADO, nome_corrompido)
            print(
                "JSON danificado preservado em: {}".format(
                    nome_corrompido
                )
            )
        except OSError:
            pass

    estado_persistente = criar_estado_padrao()
    guardar_estado()
    registar_evento_historico(
        "STATE_RESET_AFTER_LOAD_ERROR"
    )
    print(
        "Foi criado um estado limpo. "
        "Os sensores irão reconciliar os lugares no arranque."
    )


def validar_estacionamento_json(veiculo):
    """
    Confirm that parking_state.json consistently represents
    the vehicle in its assigned space.

    This validation fully replaces the previous search through .txt files.
    """
    if veiculo == "robot":
        lugar = "place_1"
        nome = "ROBOT"
    elif veiculo == "car_2":
        lugar = "place_2"
        nome = "CARRO 2"
    else:
        return False, ["veículo desconhecido"]

    problemas = []

    with estado_lock:
        dados_veiculo = dict(
            estado_persistente["vehicles"][veiculo]
        )
        dados_lugar = dict(
            estado_persistente["parking"][lugar]
        )

    if not dados_veiculo.get("inside", False):
        problemas.append(
            "{} não está marcado como dentro do parque".format(nome)
        )

    if dados_veiculo.get("location") != lugar:
        problemas.append(
            "location={} em vez de {}".format(
                dados_veiculo.get("location"),
                lugar
            )
        )

    if not dados_lugar.get("occupied", False):
        problemas.append(
            "{} não está marcado como ocupado".format(lugar)
        )

    if dados_lugar.get("vehicle") != veiculo:
        problemas.append(
            "{} tem vehicle={} em vez de {}".format(
                lugar,
                dados_lugar.get("vehicle"),
                veiculo
            )
        )

    return len(problemas) == 0, problemas



def identificar_veiculo(uid):
    uid = int(uid)
    if uid == int(estado_persistente["vehicles"]["robot"]["uid"]):
        return "robot"
    if uid == int(estado_persistente["vehicles"]["car_2"]["uid"]):
        return "car_2"
    return None


def registar_rfid(uid, veiculo, acao):
    with estado_lock:
        estado_persistente["last_rfid"] = {
            "uid": int(uid),
            "vehicle": veiculo,
            "action": acao,
            "timestamp": agora_iso()
        }
    guardar_estado()

    registar_evento_historico(
        "RFID",
        veiculo=veiculo,
        detalhes={
            "uid": int(uid),
            "action": acao
        }
    )


def bloquear_trafego(motivo):
    with estado_lock:
        estado_persistente["traffic"]["blocked"] = True
        estado_persistente["traffic"]["reason"] = motivo
    guardar_estado()


def desbloquear_trafego():
    with estado_lock:
        estado_persistente["traffic"]["blocked"] = False
        estado_persistente["traffic"]["reason"] = None
    guardar_estado()


def marcar_veiculo_fora(veiculo):
    with estado_lock:
        estado_persistente["vehicles"][veiculo]["inside"] = False
        estado_persistente["vehicles"][veiculo]["location"] = "outside"
        estado_persistente["vehicles"][veiculo]["slot"] = None
        if veiculo == "car_2":
            estado_persistente["vehicles"]["car_2"]["place_1_passage_detected"] = False
            estado_persistente["parking"]["place_2"]["reserved_by"] = None
        lugar = "place_1" if veiculo == "robot" else "place_2"
        estado_persistente["parking"][lugar]["occupied"] = False
        estado_persistente["parking"][lugar]["vehicle"] = None


def atualizar_estado_rede_json(vehicle_id, ligado):
    chave = "robot" if vehicle_id == ID_ROBOT else "car_2" if vehicle_id == ID_CARRO_2 else None
    if chave is None:
        return
    with estado_lock:
        estado_persistente["vehicles"][chave]["connected"] = bool(ligado)
        estado_persistente["vehicles"][chave]["last_seen"] = agora_iso()
    guardar_estado()


def configurar_gpio():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    sensores = [
        (TRIG_ENTRADA, ECHO_ENTRADA),
        (TRIG_CANTO, ECHO_CANTO),
        (TRIG_LUGAR_1, ECHO_LUGAR_1),
        (TRIG_LUGAR_2, ECHO_LUGAR_2)
    ]
    for trigger, echo in sensores:
        GPIO.setup(trigger, GPIO.OUT)
        GPIO.setup(echo, GPIO.IN)
        GPIO.output(trigger, GPIO.LOW)
    time.sleep(0.5)
    print("GPIO e sensores configurados.")


def medir_distancia(trigger, echo):
    GPIO.output(trigger, GPIO.LOW)
    time.sleep(0.000002)
    GPIO.output(trigger, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trigger, GPIO.LOW)
    inicio_timeout = time.monotonic()
    while GPIO.input(echo) == GPIO.LOW:
        if time.monotonic() - inicio_timeout >= TIMEOUT_ECHO:
            return None
    inicio_pulso = time.monotonic()
    inicio_timeout = time.monotonic()
    while GPIO.input(echo) == GPIO.HIGH:
        if time.monotonic() - inicio_timeout >= TIMEOUT_ECHO:
            return None
    distancia = (time.monotonic() - inicio_pulso) * 17150.0
    if distancia < 2.0 or distancia > 400.0:
        return None
    return round(distancia, 1)


def medir_todos_sensores():
    # The four HC-SR04 sensors are still triggered sequentially.
    # Reducing the global pause only increases the polling frequency;
    # no two sensors measure simultaneously.
    entrada = medir_distancia(TRIG_ENTRADA, ECHO_ENTRADA)
    time.sleep(PAUSA_ENTRE_SENSORES)
    canto = medir_distancia(TRIG_CANTO, ECHO_CANTO)
    time.sleep(PAUSA_ENTRE_SENSORES)
    lugar_1 = medir_distancia(TRIG_LUGAR_1, ECHO_LUGAR_1)
    time.sleep(PAUSA_ENTRE_SENSORES)
    lugar_2 = medir_distancia(TRIG_LUGAR_2, ECHO_LUGAR_2)
    time.sleep(PAUSA_ENTRE_SENSORES)
    return {"entrada": entrada, "canto": canto, "lugar_1": lugar_1, "lugar_2": lugar_2}


def confirmar_canto_rapido(primeira_distancia):
    """
    Quickly confirm the ROBOT's arrival at the corner sensor
    during automatic parking.

    Called only in state ROBOT_AVANCAR_ATE_CANTO.
    Does not change the normal polling interval for the remaining sensors.
    """
    if not esta_ocupado(primeira_distancia, LIMITE_CANTO_ENTRADA_CM):
        return False, None

    time.sleep(PAUSA_CONFIRMACAO_RAPIDA_CANTO)

    segunda_distancia = medir_distancia(
        TRIG_CANTO,
        ECHO_CANTO
    )

    confirmado = esta_ocupado(
        segunda_distancia,
        LIMITE_CANTO_ENTRADA_CM
    )

    return confirmado, segunda_distancia


def processar_polling_rapido_entrada_sair1():
    """
    Critical polling mode ONLY during ROBOT_SAIDA_COMANDO_ATE_ENTRADA.

    Instead of waiting to measure Entrance -> Corner -> Space1 -> Space2,
    sample the entrance sensor directly approximately every 50 ms.

    When the reading is <= 20 cm:
      1) wait 25 ms;
      2) measure the entrance again;
      3) if the reading remains <= 20 cm, send STOP immediately.

    Do not apply triple detection of a "new CAR_2" in this phase, because
    the ROBOT's own approach produces the same <= 15 cm reading.

    Return True if this critical polling cycle was processed.
    """
    primeira_distancia = medir_distancia(
        TRIG_ENTRADA,
        ECHO_ENTRADA
    )

    # FIX v37: prevent the ROBOT itself from being mistaken for CAR_2.
    # During Corner -> Entrance, the ROBOT is the vehicle approaching
    # the entrance HC-SR04. The previous rule tested <= 15 cm with
    # THREE readings BEFORE checking the ROBOT's normal arrival at <= 20 cm
    # with two readings. When the ROBOT moved directly to <= 15 cm
    # between samples, it incorrectly entered
    # AGUARDAR_RFID_CARRO_2_DURANTE_SAIDA_ROBOT.
    #
    # In this final segment, ultrasonic detection alone cannot
    # identify the vehicle. Detection at <= 20 cm is therefore treated
    # as the ROBOT's arrival stop, pending its exit RFID identification.
    # A conflict with a new CAR_2 is still detected along the
    # Space 1 -> Corner segment and during manual exit, when the ROBOT is not yet
    # near the entrance sensor. If another vehicle appears at the entrance during
    # the final segment, the route must be checked physically:
    # the sensor alone cannot distinguish the ROBOT from CAR_2.
    if esta_ocupado(
        primeira_distancia,
        LIMITE_ENTRADA_SAIR1_CM
    ):
        entrada_confirmada, segunda_distancia = (
            confirmar_entrada_sair1_rapido(primeira_distancia)
        )

        if entrada_confirmada:
            print(
                "SAIR1: entrada confirmada em polling rápido: "
                "primeira={} | segunda={} | limite={} cm. "
                "A enviar STOP imediatamente.".format(
                    texto_distancia(primeira_distancia),
                    texto_distancia(segunda_distancia),
                    LIMITE_ENTRADA_SAIR1_CM
                )
            )

            if not enviar_comando_robot("S"):
                return True

            with estado_lock:
                estado_persistente["vehicles"]["robot"]["location"] = (
                    "entrance"
                )
            guardar_estado()

            enviar_evento(
                ID_ROBOT,
                "RFID_EXIT_REQUIRED",
                {"LOCATION": "ENTRANCE", "MODE": "SAIR1"}
            )
            mudar_estado(AGUARDAR_RFID_SAIDA_ROBOT)
            return True

    if tempo_no_estado() >= TEMPO_MAXIMO_SAIDA_ATE_ENTRADA:
        print(
            "ERRO SAIR1: o ROBOT não chegou ao sensor da entrada "
            "dentro do tempo máximo."
        )
        enviar_comando_robot("S")
        mudar_estado(ERRO_PERCURSO)
        return True

    time.sleep(PAUSA_POLLING_ENTRADA_SAIR1)
    return True


def confirmar_entrada_conflito_tripla(primeira_distancia):
    """One normal entrance reading plus two more, spaced 25 ms apart.

    Do not attempt to identify which vehicle reflects the ultrasound:
    RFID identifies the vehicle after the ROBOT stops.
    """
    leituras = [primeira_distancia]
    if not esta_ocupado(primeira_distancia, LIMITE_ENTRADA_CM):
        return False, leituras
    for _ in range(2):
        time.sleep(PAUSA_CONFIRMACAO_TRIPLA_ENTRADA_CONFLITO)
        nova = medir_distancia(TRIG_ENTRADA, ECHO_ENTRADA)
        leituras.append(nova)
        if not esta_ocupado(nova, LIMITE_ENTRADA_CM):
            return False, leituras
    return True, leituras


def suspender_saida_robot_por_veiculo_na_entrada(leituras):
    """Phase-preserving stop: the Robot stores its motor state and timer on H."""
    global saida_robot_estado_interrompido
    global saida_robot_pausa_confirmada
    global saida_robot_retomada_confirmada
    fase = estado_sistema
    if fase not in (
        ROBOT_SAIDA_COMANDO_ATE_CANTO,
        ROBOT_SAIDA_COMANDO_ATE_ENTRADA,
        ROBOT_A_SAIR_ATE_ENTRADA,
    ):
        return False
    if not cliente_ligado(ID_ROBOT):
        print("ALERTA: entrada ocupada mas ROBOT desligado; pare-o manualmente.")
        bloquear_trafego("CONFLITO_ENTRADA_ROBOT_SEM_COMUNICACAO")
        mudar_estado(ERRO_PERCURSO)
        return True

    # H pauses while preserving the motor state, including an unfinished
    # turn. Unlike S, this allows J to resume that state.
    saida_robot_pausa_confirmada = False
    saida_robot_retomada_confirmada = False
    if not enviar_comando_robot("H"):
        print("FALHA: não foi possível suspender a saída. NÃO MOVA CARRO 2.")
        bloquear_trafego("CONFLITO_ENTRADA_SEM_STOP_CONFIRMADO")
        mudar_estado(ERRO_PERCURSO)
        return True
    saida_robot_estado_interrompido = fase
    bloquear_trafego("ROBOT_SUSPENSO_RFID_CARRO_2_ENTRADA")
    registar_evento_historico(
        "ROBOT_EXIT_PAUSE_REQUESTED", veiculo="robot",
        detalhes={"exit_phase": fase, "entry_readings_cm": leituras}
    )
    enviar_evento(
        ID_CARRO_2, "RFID_SAFE_PLACE_REQUIRED",
        {"REASON": "ROBOT_EXIT_INTERRUPTED", "ROBOT_PHASE": fase}
    )
    print("CONFLITO NA ENTRADA: leituras {} <= {} cm."
          .format(leituras, LIMITE_ENTRADA_CM))
    print("ROBOT a suspender saída. Aguarde confirmação de imobilização ")
    print("e apresente RFID do CARRO 2, ANTES de o mover ao SAFE PLACE.")
    mudar_estado(AGUARDAR_RFID_CARRO_2_DURANTE_SAIDA_ROBOT)
    return True


def tentar_detetar_conflito_entrada_durante_saida(primeira_distancia):
    if not esta_ocupado(primeira_distancia, LIMITE_ENTRADA_CM):
        return False
    confirmado, leituras = confirmar_entrada_conflito_tripla(primeira_distancia)
    if confirmado:
        return suspender_saida_robot_por_veiculo_na_entrada(leituras)
    return False


def confirmar_entrada_start_rapido(primeira_distancia):
    """
    Confirm the ROBOT's arrival at the entrance in the new START scenario.

    First reading: normal sensor cycle.
    If it is <= LIMITE_ENTRADA_CM (15 cm):
      - wait 25 ms;
      - measure ONLY the entrance sensor again;
      - confirm if the second reading is also <= 15 cm.
    """
    if not esta_ocupado(
        primeira_distancia,
        LIMITE_ENTRADA_CM
    ):
        return False, None

    time.sleep(PAUSA_CONFIRMACAO_RAPIDA_ENTRADA_START)

    segunda_distancia = medir_distancia(
        TRIG_ENTRADA,
        ECHO_ENTRADA
    )

    confirmado = esta_ocupado(
        segunda_distancia,
        LIMITE_ENTRADA_CM
    )

    return confirmado, segunda_distancia


def confirmar_entrada_retorno_rapido(primeira_distancia):
    """
    Quickly confirm the ROBOT's arrival at the entrance while
    returning from the SAFE PLACE.

    First reading: use the normal sensor cycle.
    If it is <= 15 cm:
      - wait 25 ms;
      - measure only the entrance sensor again;
      - confirm the stop if the second reading is also <= 15 cm.
    """
    if not esta_ocupado(
        primeira_distancia,
        LIMITE_ENTRADA_CM
    ):
        return False, None

    time.sleep(PAUSA_CONFIRMACAO_RAPIDA_ENTRADA_RETORNO)

    segunda_distancia = medir_distancia(
        TRIG_ENTRADA,
        ECHO_ENTRADA
    )

    confirmado = esta_ocupado(
        segunda_distancia,
        LIMITE_ENTRADA_CM
    )

    return confirmado, segunda_distancia


def confirmar_entrada_sair1_rapido(primeira_distancia):
    """
    Quickly confirm the ROBOT's arrival at the entrance during SAIR1.

    The first reading comes from the normal cycle. If it is <= 20 cm,
    wait 25 ms and immediately measure only the entrance sensor
    again.
    """
    if not esta_ocupado(
        primeira_distancia,
        LIMITE_ENTRADA_SAIR1_CM
    ):
        return False, None

    time.sleep(PAUSA_CONFIRMACAO_RAPIDA_ENTRADA_SAIR1)

    segunda_distancia = medir_distancia(
        TRIG_ENTRADA,
        ECHO_ENTRADA
    )

    confirmado = esta_ocupado(
        segunda_distancia,
        LIMITE_ENTRADA_SAIR1_CM
    )

    return confirmado, segunda_distancia


def confirmar_lugar_1_rapido(primeira_distancia):
    """
    Quickly confirm the ROBOT's arrival at space 1
    during automatic parking, using the 15 cm threshold.

    Called only in state ROBOT_AVANCAR_ATE_LUGAR_1.
    Does not change the normal polling interval for the remaining sensors.
    """
    if not esta_ocupado(primeira_distancia, LIMITE_LUGAR_1_ESTACIONAMENTO_CM):
        return False, None

    time.sleep(PAUSA_CONFIRMACAO_RAPIDA_LUGAR_1)

    segunda_distancia = medir_distancia(
        TRIG_LUGAR_1,
        ECHO_LUGAR_1
    )

    confirmado = esta_ocupado(
        segunda_distancia,
        LIMITE_LUGAR_1_ESTACIONAMENTO_CM
    )

    return confirmado, segunda_distancia


def esta_ocupado(distancia, limite):
    return distancia is not None and distancia <= limite


def esta_livre(distancia, limite):
    return distancia is not None and distancia > limite


def lugar_estacionamento_livre(distancia, limite):
    return distancia is None or distancia > limite


def lugar_2_livre_confirmavel(distancia):
    """Confirm space 2 as FREE only with a valid measurement >= 15 cm."""
    return distancia is not None and distancia >= LIMITE_LUGAR_2_LIVRE_CM


def texto_distancia(distancia):
    return "SEM LEITURA" if distancia is None else "{:.1f} cm".format(distancia)


def estado_ocupacao(distancia, limite):
    if distancia is None:
        return "SEM LEITURA"
    return "OCUPADO" if distancia <= limite else "LIVRE"


def estado_ocupacao_lugar_2(distancia):
    if distancia is None:
        return "SEM LEITURA"
    if distancia <= LIMITE_LUGAR_2_CM:
        return "OCUPADO"
    if lugar_2_livre_confirmavel(distancia):
        return "LIVRE"
    return "INDETERMINADO (10-15 cm)"


def atualizar_contadores(distancias):
    global contador_lugar_1_ocupado, contador_lugar_1_livre
    global contador_lugar_2_ocupado, contador_lugar_2_livre
    global contador_entrada, contador_entrada_livre
    global contador_entrada_saida_robot
    global contador_entrada_sair1
    lugar_1 = distancias["lugar_1"]
    lugar_2 = distancias["lugar_2"]
    entrada = distancias["entrada"]
    if esta_ocupado(lugar_1, LIMITE_LUGAR_1_CM):
        contador_lugar_1_ocupado += 1
        contador_lugar_1_livre = 0
    elif lugar_estacionamento_livre(lugar_1, LIMITE_LUGAR_1_CM):
        contador_lugar_1_livre += 1
        contador_lugar_1_ocupado = 0
    else:
        contador_lugar_1_ocupado = 0
        contador_lugar_1_livre = 0
    if esta_ocupado(lugar_2, LIMITE_LUGAR_2_CM):
        contador_lugar_2_ocupado += 1
        contador_lugar_2_livre = 0
    elif lugar_2_livre_confirmavel(lugar_2):
        contador_lugar_2_livre += 1
        contador_lugar_2_ocupado = 0
    else:
        contador_lugar_2_ocupado = 0
        contador_lugar_2_livre = 0
    if esta_ocupado(entrada, LIMITE_ENTRADA_CM):
        contador_entrada += 1
        contador_entrada_livre = 0
    elif esta_livre(entrada, LIMITE_ENTRADA_CM):
        contador_entrada_livre += 1
        contador_entrada = 0
    else:
        contador_entrada = 0
        contador_entrada_livre = 0

    # Separate entrance confirmation for the Robot's automatic/legacy
    # exit. Retains the 20 cm threshold.
    if esta_ocupado(
        entrada,
        LIMITE_ENTRADA_SAIDA_ROBOT_CM
    ):
        contador_entrada_saida_robot += 1
    else:
        contador_entrada_saida_robot = 0

    # Dedicated entrance confirmation for the SAIR1 command.
    # A reading <= 25 cm counts as detection.
    if esta_ocupado(
        entrada,
        LIMITE_ENTRADA_SAIR1_CM
    ):
        contador_entrada_sair1 += 1
    else:
        contador_entrada_sair1 = 0


def reconciliar_json_com_sensores():
    global sensores_inicializados
    if sensores_inicializados:
        return
    l1 = contador_lugar_1_ocupado >= CONFIRMACOES_SENSOR or contador_lugar_1_livre >= CONFIRMACOES_SENSOR
    l2 = contador_lugar_2_ocupado >= CONFIRMACOES_SENSOR or contador_lugar_2_livre >= CONFIRMACOES_SENSOR
    if not l1 or not l2:
        return
    with estado_lock:
        if contador_lugar_1_ocupado >= CONFIRMACOES_SENSOR:
            estado_persistente["vehicles"]["robot"]["inside"] = True
            estado_persistente["vehicles"]["robot"]["location"] = "place_1"
            estado_persistente["parking"]["place_1"]["occupied"] = True
            estado_persistente["parking"]["place_1"]["vehicle"] = "robot"
        else:
            marcar_veiculo_fora("robot")
        if contador_lugar_2_ocupado >= CONFIRMACOES_SENSOR:
            estado_persistente["vehicles"]["car_2"]["inside"] = True
            estado_persistente["vehicles"]["car_2"]["location"] = "place_2"
            estado_persistente["vehicles"]["car_2"]["slot"] = 3
            estado_persistente["parking"]["place_2"]["occupied"] = True
            estado_persistente["parking"]["place_2"]["vehicle"] = "car_2"
            estado_persistente["parking"]["place_2"]["reserved_by"] = None
        else:
            marcar_veiculo_fora("car_2")
        estado_persistente["traffic"]["blocked"] = False
        estado_persistente["traffic"]["reason"] = None
    guardar_estado()
    sensores_inicializados = True
    print("\n" + "=" * 72)
    print("ESTADO JSON SINCRONIZADO COM OS SENSORES")
    print("=" * 72)


def mudar_estado(novo_estado):
    global estado_sistema, instante_inicio_estado
    if estado_sistema == novo_estado:
        return

    estado_anterior = estado_sistema
    estado_sistema = novo_estado
    instante_inicio_estado = time.monotonic()

    print("\n" + "=" * 72)
    print("NOVO ESTADO: {}".format(novo_estado))
    print("=" * 72)

    registar_evento_historico(
        "STATE_CHANGED",
        detalhes={
            "from": estado_anterior,
            "to": novo_estado
        }
    )

    transmitir_evento_infraestrutura(
        "STATE_CHANGED",
        {"STATE": novo_estado}
    )


def tempo_no_estado():
    return time.monotonic() - instante_inicio_estado


def pode_mostrar_mensagem():
    global ultima_mensagem
    agora = time.monotonic()
    if agora - ultima_mensagem < INTERVALO_MENSAGEM:
        return False
    ultima_mensagem = agora
    return True


def mostrar_distancias(distancias):
    global ultimo_resumo
    agora = time.monotonic()
    if agora - ultimo_resumo < INTERVALO_RESUMO:
        return
    ultimo_resumo = agora
    with clientes_lock:
        robot_ligado = ID_ROBOT in clientes
        carro_2_ligado = ID_CARRO_2 in clientes
    print("\n" + "-" * 88)
    print("ESTADO: {}".format(estado_sistema))
    print("Rede    : ROBOT={} | CAR_2={}".format(
        "LIGADO" if robot_ligado else "DESLIGADO",
        "LIGADO" if carro_2_ligado else "DESLIGADO"))
    print("Entrada : {:>12}".format(texto_distancia(distancias["entrada"])))
    print("Canto   : {:>12}".format(texto_distancia(distancias["canto"])))
    print("Lugar 1 : {:>12} | {}".format(texto_distancia(distancias["lugar_1"]), estado_ocupacao(distancias["lugar_1"], LIMITE_LUGAR_1_CM)))
    print("Lugar 2 : {:>12} | {}".format(texto_distancia(distancias["lugar_2"]), estado_ocupacao_lugar_2(distancias["lugar_2"])))
    with estado_lock:
        slot_robot = estado_persistente["vehicles"]["robot"].get("slot")
        slot_car2 = estado_persistente["vehicles"]["car_2"].get("slot")
        reservado_l2 = estado_persistente["parking"]["place_2"].get("reserved_by")
    print("Slots   : ROBOT={} | CAR_2={} | Reserva L2={}".format(
        "-" if slot_robot is None else slot_robot,
        "-" if slot_car2 is None else slot_car2,
        reservado_l2 or "nenhuma"
    ))
    print("Tráfego : {}".format("BLOQUEADO — " + str(estado_persistente["traffic"]["reason"]) if estado_persistente["traffic"]["blocked"] else "LIVRE"))
    print("-" * 88)


def cliente_ligado(vehicle_id):
    with clientes_lock:
        return normalizar_id_cliente(vehicle_id) in clientes


def iniciar_servidor_tcp():
    global servidor_tcp
    servidor_tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    servidor_tcp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    servidor_tcp.bind((HOST, PORTA_TCP))
    servidor_tcp.listen(MAX_CLIENTES_PENDENTES)
    servidor_tcp.settimeout(1.0)
    threading.Thread(target=aceitar_clientes, daemon=True).start()
    print("Servidor TCP multi-cliente ativo em {}:{}.".format(HOST, PORTA_TCP))


def aceitar_clientes():
    while programa_ativo:
        try:
            sock, endereco = servidor_tcp.accept()
            sock.settimeout(TIMEOUT_HELLO)
            print("Nova ligação TCP de {}. A aguardar HELLO...".format(endereco))
            threading.Thread(target=gerir_cliente, args=(sock, endereco), daemon=True).start()
        except socket.timeout:
            continue
        except OSError as erro:
            if programa_ativo:
                print("Erro ao aceitar cliente: {}".format(erro))
            break


def receber_linhas(sock, buffer):
    dados = sock.recv(1024)
    if not dados:
        raise ConnectionResetError("Cliente fechou a ligação.")
    buffer += dados.decode("utf-8", errors="ignore")
    linhas = []
    while "\n" in buffer:
        linha, buffer = buffer.split("\n", 1)
        linha = linha.strip()
        if linha:
            linhas.append(linha)
    return linhas, buffer


def gerir_cliente(sock, endereco):
    vehicle_id = None
    buffer = ""
    try:
        while programa_ativo and vehicle_id is None:
            linhas, buffer = receber_linhas(sock, buffer)
            for mensagem in linhas:
                tipo, campos = parse_mensagem(mensagem)
                if tipo != "HELLO":
                    enviar_socket_direto(sock, "ERROR|CODE=HELLO_REQUIRED")
                    continue
                pedido_id = normalizar_id_cliente(campos.get("ID"))
                pedido_tipo = campos.get("TYPE", "").upper()
                if campos.get("PROTOCOL") != "2":
                    # Older firmware does not check TO and could execute
                    # broadcast events intended for other vehicles.
                    # Rejecting the connection is safer than keeping an
                    # incompatible client connected to the shared channel.
                    enviar_socket_direto(
                        sock, "ERROR|CODE=ADDRESS_FILTER_REQUIRED|PROTOCOL=2"
                    )
                    return
                if pedido_id not in IDS_PERMITIDOS:
                    enviar_socket_direto(sock, "ERROR|CODE=INVALID_ID")
                    return
                tipo_esperado = IDS_PERMITIDOS[pedido_id]
                if pedido_tipo and pedido_tipo != tipo_esperado:
                    enviar_socket_direto(sock, "ERROR|CODE=INVALID_VEHICLE_TYPE")
                    return
                vehicle_id = pedido_id
                registar_cliente(vehicle_id, tipo_esperado, sock, endereco)
                sock.settimeout(TIMEOUT_CLIENTE)
                enviar_para_veiculo(vehicle_id, "WELCOME|ID={}|SERVER=RPI|PROTOCOL=2".format(vehicle_id))
                enviar_para_veiculo(vehicle_id, "EVENT|SOURCE=INFRASTRUCTURE|TYPE=SYSTEM_STATE|STATE={}".format(estado_sistema))
                break
        while programa_ativo and vehicle_id is not None:
            try:
                linhas, buffer = receber_linhas(sock, buffer)
                for mensagem in linhas:
                    atualizar_last_seen(vehicle_id)
                    processar_mensagem_cliente(vehicle_id, mensagem)
            except socket.timeout:
                continue
    except (ConnectionResetError, BrokenPipeError, OSError) as erro:
        print("{} desligado: {}".format(vehicle_id if vehicle_id else endereco, erro))
    except Exception as erro:
        print("Erro na thread do cliente {}: {}".format(vehicle_id if vehicle_id else endereco, erro))
    finally:
        if vehicle_id is not None:
            remover_cliente(vehicle_id, sock)
        try:
            sock.close()
        except OSError:
            pass


def registar_cliente(vehicle_id, vehicle_type, sock, endereco):
    with clientes_lock:
        antigo = clientes.get(vehicle_id)
        clientes[vehicle_id] = {
            "socket": sock,
            "address": endereco,
            "type": vehicle_type,
            "connected_at": time.monotonic(),
            "last_seen": time.monotonic(),
            "send_lock": threading.Lock()
        }
    if antigo is not None:
        try:
            antigo["socket"].close()
        except OSError:
            pass
    atualizar_estado_rede_json(vehicle_id, True)
    print("\n{} REGISTADO | tipo={} | endereço={}".format(vehicle_id, vehicle_type, endereco))
    outro = ID_CARRO_2 if vehicle_id == ID_ROBOT else ID_ROBOT
    encaminhar_v2v(vehicle_id, outro, "VEHICLE_CONNECTED")


def remover_cliente(vehicle_id, socket_esperado=None):
    with clientes_lock:
        atual = clientes.get(vehicle_id)
        if atual is None:
            return
        if socket_esperado is not None and atual["socket"] is not socket_esperado:
            return
        removido = clientes.pop(vehicle_id)
    try:
        removido["socket"].close()
    except OSError:
        pass
    atualizar_estado_rede_json(vehicle_id, False)
    print("{} removido do registo TCP.".format(vehicle_id))
    outro = ID_CARRO_2 if vehicle_id == ID_ROBOT else ID_ROBOT
    encaminhar_v2v(vehicle_id, outro, "VEHICLE_DISCONNECTED")


def fechar_todos_clientes():
    with clientes_lock:
        ids = list(clientes.keys())
    for vehicle_id in ids:
        remover_cliente(vehicle_id)


def enviar_socket_direto(sock, mensagem):
    sock.sendall((mensagem.rstrip("\n") + "\n").encode("utf-8"))


# All connected vehicles receive the SAME TCP line (logical broadcast).
# Each ESP32 filters TO=... BEFORE processing COMMAND/EVENT/V2V.
# This is not IP broadcast: TCP still uses an individual connection for each vehicle.
MAPA_COMANDOS_ROBOT = {
    "F": "FORWARD",
    "S": "STOP",
    "R": "TURN_RIGHT",
    "P": "SAFE_PLACE",
    "B": "RETURN_FROM_SAFE_PLACE",
    "U": "TURN_AROUND_EXIT",
    "L": "TURN_LEFT_EXIT",
    "X": "EXIT_TO_START",
    "H": "PAUSE_EXIT",
    "J": "RESUME_EXIT"
}


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


def enviar_para_veiculo(vehicle_id, mensagem):
    """
    Perform logical broadcast over all authenticated TCP connections.

    Even for TO=ROBOT, the other ESP32 clients receive the frame and ignore it
    after checking TO. Preserve the original return value: True only
    when the intended recipient's socket has received the frame.

    Safety: send to the recipient FIRST, especially for STOP,
    to avoid delays caused by other vehicles' congested sockets.
    """
    destino = normalizar_id_cliente(vehicle_id)
    if destino != "ALL" and destino not in IDS_PERMITIDOS:
        print("Destino TCP inválido: {}".format(destino))
        return False

    try:
        frame = preparar_mensagem_enderecada(destino, mensagem)
    except ValueError as erro:
        print("Mensagem não enviada: {}".format(erro))
        return False

    with clientes_lock:
        # Take a snapshot: never hold the global lock during sendall().
        sessao_por_id = list(clientes.items())

    if destino != "ALL" and not any(vid == destino for vid, _ in sessao_por_id):
        print("Não foi possível enviar para {}: veículo desligado.".format(destino))
        return False

    # In frames addressed to one vehicle, place the recipient first.
    if destino != "ALL":
        sessao_por_id.sort(key=lambda par: par[0] != destino)

    dados = (frame + "\n").encode("utf-8")
    entrega_destino = False
    entregas = 0
    falhas = []

    for destinatario_socket, sessao in sessao_por_id:
        try:
            with sessao["send_lock"]:
                sessao["socket"].sendall(dados)
            entregas += 1
            if destinatario_socket == destino:
                entrega_destino = True
        except (BrokenPipeError, ConnectionResetError, OSError) as erro:
            print("Erro no broadcast para {}: {}".format(
                destinatario_socket, erro
            ))
            falhas.append((destinatario_socket, sessao["socket"]))

    for destinatario_socket, sock_antigo in falhas:
        remover_cliente(destinatario_socket, sock_antigo)

    print(">>> BROADCAST TCP [{} clientes] {}".format(entregas, frame))
    return entregas > 0 if destino == "ALL" else entrega_destino


def enviar_comando_autonomo(vehicle_id, comando):
    """
    Send an action to any autonomous vehicle registered in the protocol.
    The infrastructure still decides WHEN to send it; the vehicle determines
    whether to execute it by comparing the frame's TO with its VEHICLE_ID.
    """
    destino = normalizar_id_cliente(vehicle_id)
    if IDS_PERMITIDOS.get(destino) != TIPO_ROBOT:
        print("Comando recusado: {} não é um robot autónomo registado.".format(
            destino
        ))
        return False
    if comando not in MAPA_COMANDOS_ROBOT:
        print("Comando autónomo desconhecido: {}".format(comando))
        return False
    return enviar_para_veiculo(destino, comando)


def enviar_comando_robot(comando):
    # Preserve the previous API for ALL existing scenarios.
    return enviar_comando_autonomo(ID_ROBOT, comando)


def enviar_evento(vehicle_id, tipo_evento, campos=None):
    partes = ["EVENT", "SOURCE=INFRASTRUCTURE", "TYPE={}".format(tipo_evento)]
    if campos:
        for chave, valor in campos.items():
            partes.append("{}={}".format(str(chave).upper(), valor))
    return enviar_para_veiculo(vehicle_id, "|".join(partes))


def transmitir_evento_infraestrutura(tipo_evento, campos=None):
    # Actual global event: send the TO=ALL frame once to each
    # TCP session. Avoid N x N duplication when using logical broadcast.
    return enviar_evento("ALL", tipo_evento, campos)


def encaminhar_v2v(origem, destino, evento, campos=None):
    origem = normalizar_id_cliente(origem)
    destino = normalizar_id_cliente(destino)
    partes = ["V2V", "FROM={}".format(origem), "TO={}".format(destino), "EVENT={}".format(evento)]
    if campos:
        for chave, valor in campos.items():
            if str(chave).upper() not in ("FROM", "TO", "EVENT", "TYPE"):
                partes.append("{}={}".format(str(chave).upper(), valor))
    return enviar_para_veiculo(destino, "|".join(partes))


def atualizar_last_seen(vehicle_id):
    with clientes_lock:
        cliente = clientes.get(vehicle_id)
        if cliente is not None:
            cliente["last_seen"] = time.monotonic()


def verificar_timeouts_clientes():
    global ultima_verificacao_clientes
    agora = time.monotonic()
    if agora - ultima_verificacao_clientes < INTERVALO_VERIFICACAO_CLIENTES:
        return
    ultima_verificacao_clientes = agora
    expirados = []
    with clientes_lock:
        for vehicle_id, cliente in clientes.items():
            if agora - cliente["last_seen"] > TIMEOUT_HEARTBEAT:
                expirados.append((vehicle_id, cliente["socket"]))
    for vehicle_id, sock in expirados:
        print("{} sem heartbeat há mais de {:.0f} s.".format(vehicle_id, TIMEOUT_HEARTBEAT))
        remover_cliente(vehicle_id, sock)


def processar_mensagem_cliente(vehicle_id, mensagem):
    tipo, campos = parse_mensagem(mensagem)
    print("[{}] {}".format(vehicle_id, mensagem))
    if tipo == "PING":
        enviar_para_veiculo(vehicle_id, "PONG|ID={}|TIME={}".format(vehicle_id, agora_iso()))
    elif tipo == "PONG":
        pass
    elif tipo == "STATUS":
        estado_recebido = campos.get("STATE", "UNKNOWN")
        localizacao_recebida = campos.get("LOCATION")

        print(
            "Estado comunicado por {}: {} | localização={}".format(
                vehicle_id,
                estado_recebido,
                localizacao_recebida or "UNKNOWN"
            )
        )

        chave_veiculo = (
            "robot" if vehicle_id == ID_ROBOT
            else "car_2" if vehicle_id == ID_CARRO_2
            else None
        )

        if chave_veiculo == "car_2" and localizacao_recebida:
            # IMPORTANT: Car 2 is moved manually and has no onboard
            # position sensors. Its STATUS report only represents the
            # last phase known to its firmware, NOT physical confirmation
            # of its location. For example, ENTRY_AUTHORIZED reports
            # PARKING/ENTRANCE even when the Raspberry Pi has already stored
            # location='parking' to monitor arrival at space 2.
            #
            # Never overwrite the controller's authoritative state with
            # ESP32 STATUS: otherwise, the background monitor stops
            # validating arrival and SAIR2 stops working.
            with estado_lock:
                localizacao_autoritativa = (
                    estado_persistente["vehicles"]["car_2"]["location"]
                )
            if localizacao_autoritativa != localizacao_recebida.lower():
                print(
                    "Telemetria CARRO 2: LOCATION={} (reportada) | "
                    "location={} (controlada pelo Raspberry; mantida).".format(
                        localizacao_recebida,
                        localizacao_autoritativa
                    )
                )
        elif chave_veiculo == "robot" and localizacao_recebida:
            # During an exit suspension, the controller retains the
            # logical location. STATUS is telemetry, not authorization.
            if estado_sistema not in (
                AGUARDAR_RFID_CARRO_2_DURANTE_SAIDA_ROBOT,
                AGUARDAR_ENTRADA_LIVRE_PARA_RETOMAR_SAIDA_ROBOT,
                AGUARDAR_ACK_RETOMA_SAIDA_ROBOT,
            ):
                with estado_lock:
                    estado_persistente["vehicles"]["robot"]["location"] = (
                        localizacao_recebida.lower()
                    )
                guardar_estado()
    elif tipo == "ACK":
        global saida_robot_pausa_confirmada
        global saida_robot_retomada_confirmada
        global robot_avanco_final_confirmado
        print("ACK recebido de {}: {}".format(vehicle_id, campos))
        if vehicle_id == ID_ROBOT and campos.get("STATUS") == "COMPLETED":
            with estado_lock:
                if campos.get("ACTION") == "PAUSE_EXIT":
                    saida_robot_pausa_confirmada = True
                elif campos.get("ACTION") == "RESUME_EXIT":
                    saida_robot_retomada_confirmada = True
                elif campos.get("ACTION") == "EXIT_TO_START":
                    robot_avanco_final_confirmado = True

        if (
            vehicle_id == ID_ROBOT
            and campos.get("ACTION") == "RETURN_FROM_SAFE_PLACE"
            and campos.get("STATUS") == "COMPLETED"
        ):
            print(
                "Robot regressou do safe place à posição inicial."
            )
            with estado_lock:
                estado_persistente["vehicles"]["robot"]["location"] = (
                    "entrance"
                )
            guardar_estado()
            enviar_evento(
                ID_ROBOT,
                "RETURN_COMPLETED",
                {"LOCATION": "ENTRANCE"}
            )
    elif tipo == "EVENT":
        destino = normalizar_id_cliente(campos.get("TO"))
        evento = campos.get("TYPE", "UNKNOWN")
        if destino in IDS_PERMITIDOS and destino != vehicle_id:
            encaminhar_v2v(vehicle_id, destino, evento, campos)
    elif tipo == "V2V":
        destino = normalizar_id_cliente(campos.get("TO"))
        evento = campos.get("EVENT", campos.get("TYPE", "UNKNOWN"))
        if destino not in IDS_PERMITIDOS or destino == vehicle_id:
            enviar_para_veiculo(vehicle_id, "ERROR|CODE=INVALID_V2V_DESTINATION")
        else:
            encaminhar_v2v(vehicle_id, destino, evento, campos)
    else:
        enviar_para_veiculo(vehicle_id, "ERROR|CODE=UNKNOWN_MESSAGE|TYPE={}".format(tipo))


def thread_leitura_rfid():
    leitor = SimpleMFRC522()
    ultimo_uid = None
    instante_ultimo_uid = 0.0
    while programa_ativo:
        try:
            uid, texto = leitor.read()
            uid = int(uid)
            agora = time.monotonic()
            if uid != ultimo_uid or agora - instante_ultimo_uid >= 2.0:
                fila_rfid.put((uid, texto))
                ultimo_uid = uid
                instante_ultimo_uid = agora
            time.sleep(0.2)
        except Exception as erro:
            if programa_ativo:
                print("Erro RFID: {}".format(erro))
            time.sleep(0.5)


def iniciar_thread_rfid():
    threading.Thread(target=thread_leitura_rfid, daemon=True).start()
    print("Leitor RFID iniciado.")


def obter_leitura_rfid():
    try:
        return fila_rfid.get_nowait()
    except queue.Empty:
        return None, None


def veiculo_no_safe_place(veiculo):
    with estado_lock:
        return (
            estado_persistente["vehicles"][veiculo]["location"]
            == "safe_place"
        )


def atualizar_slots_carro_2_em_background(distancias):
    """Track CAR_2 progression in parallel with START.

    Marking SLOT 3 does not confirm parking: it only
    permits the ROBOT to start parking in space 1, provided
    the corner and space 1 sensors physically indicate a clear path.
    Space 2 remains reserved, and actual detection in space 2 is still
    required to set inside=True / occupied=True.

    Observable sequence:
    RFID validated -> Slot 0 (entrance still occupied)
    Entrance clear -> Slot 1
    Confirmed corner occupancy -> Slot 2
    Space 1 sensor occupied and then clear -> Slot 3.
    """
    global contador_canto_slot_2_ocupado, contador_canto_slot_2_livre

    with estado_lock:
        carro = estado_persistente["vehicles"]["car_2"]
        localizacao = carro["location"]
        slot = carro.get("slot")
        passou_lugar_1 = bool(carro.get("place_1_passage_detected"))

    if localizacao != "parking" or slot is None:
        contador_canto_slot_2_ocupado = 0
        contador_canto_slot_2_livre = 0
        return

    canto = distancias["canto"]
    lugar_1 = distancias["lugar_1"]
    if esta_ocupado(canto, LIMITE_CANTO_ENTRADA_CM):
        contador_canto_slot_2_ocupado += 1
        contador_canto_slot_2_livre = 0
    elif esta_livre(canto, LIMITE_CANTO_ENTRADA_CM):
        contador_canto_slot_2_livre += 1
        contador_canto_slot_2_ocupado = 0
    else:
        contador_canto_slot_2_ocupado = 0
        contador_canto_slot_2_livre = 0

    if slot == 0 and contador_entrada_livre >= CONFIRMACOES_SENSOR:
        with estado_lock:
            estado_persistente["vehicles"]["car_2"]["slot"] = 1
        guardar_estado()
        registar_evento_historico("SLOT_CHANGED", "car_2", {"slot": 1})
        enviar_evento(ID_CARRO_2, "SLOT_CHANGED", {"SLOT": 1})
        print("SLOTS: CARRO 2 passou da entrada para SLOT 1; SLOT 0 livre para o START.")
        slot = 1

    if slot == 1 and contador_canto_slot_2_ocupado >= CONFIRMACOES_SLOT:
        with estado_lock:
            estado_persistente["vehicles"]["car_2"]["slot"] = 2
        guardar_estado()
        registar_evento_historico("SLOT_CHANGED", "car_2", {"slot": 2})
        enviar_evento(ID_CARRO_2, "SLOT_CHANGED", {"SLOT": 2})
        print("SLOTS: CARRO 2 identificado no CANTO -> SLOT 2.")
        slot = 2

    if slot == 2 and not passou_lugar_1 and (
            contador_lugar_1_ocupado >= CONFIRMACOES_SLOT):
        with estado_lock:
            estado_persistente["vehicles"]["car_2"]["place_1_passage_detected"] = True
        guardar_estado()
        print("SLOTS: CARRO 2 detetado no sensor Lugar 1; aguarda libertação desse sensor.")
        passou_lugar_1 = True

    # Detection at space 1 alone does NOT authorize the ROBOT to advance past
    # that sensor. Confirm that both presence detections have cleared.
    if (slot == 2 and passou_lugar_1
            and contador_lugar_1_livre >= CONFIRMACOES_SLOT
            and contador_canto_slot_2_livre >= CONFIRMACOES_SLOT
            and esta_livre(lugar_1, LIMITE_LUGAR_1_CM)
            and esta_livre(canto, LIMITE_CANTO_ENTRADA_CM)):
        with estado_lock:
            estado_persistente["vehicles"]["car_2"]["slot"] = 3
        guardar_estado()
        registar_evento_historico(
            "SLOT_CHANGED", "car_2",
            {"slot": 3, "place_2_reserved": True,
             "parking_confirmed": False}
        )
        enviar_evento(
            ID_CARRO_2, "SLOT_CHANGED",
            {"SLOT": 3, "PLACE": "PLACE_2", "STATUS": "EN_ROUTE"}
        )
        # Still a TCP broadcast, but addressed only to the ROBOT.
        # This is a NOTIFICATION; actual authorization still depends
        # on RFID and physical route validation by the Raspberry Pi.
        enviar_evento(
            ID_ROBOT, "SLOT_1_2_RELEASED",
            {"BY": "CAR_2", "NEXT": "RFID_ENTRY", "PLACE_2": "RESERVED"}
        )
        print(
            "SLOTS: CARRO 2 passou o sensor do Lugar 1 e entrou no SLOT 3. "
            "Lugar 2 continua reservado (AINDA NÃO ESTACIONADO). "
            "ROBOT pode apresentar RFID quando estiver na entrada e "
            "o corredor continuar livre."
        )
        with estado_lock:
            robot_entrada = (
                estado_persistente["vehicles"]["robot"]["location"]
                == "entrance"
            )
        if robot_entrada and estado_sistema == AGUARDAR_RFID_ENTRADA:
            desbloquear_trafego()
        return


def corredor_robot_livre_para_rfid(distancias):
    """Allow Robot entry only after Car 2 passes space 1 and the route is clear."""
    with estado_lock:
        car2 = estado_persistente["vehicles"]["car_2"]
        location = car2["location"]
        slot = car2.get("slot")
        reservado = (estado_persistente["parking"]["place_2"].get("reserved_by")
                     == "car_2")
    if location != "parking":
        return True
    return (
        reservado and slot == 3
        and contador_lugar_1_livre >= CONFIRMACOES_SLOT
        and esta_livre(distancias["lugar_1"], LIMITE_LUGAR_1_CM)
        and contador_canto_slot_2_livre >= CONFIRMACOES_SLOT
        and esta_livre(distancias["canto"], LIMITE_CANTO_ENTRADA_CM)
    )


def finalizar_estacionamento_carro_2_em_background():
    """
    Monitor CAR_2 parking even after the
    ROBOT resumes the START command.

    This is necessary because the system uses one main state.
    When CAR_2 clears the entrance, the main state changes to
    ROBOT_START_ATE_ENTRADA, but space 2 is still monitored.
    """
    with estado_lock:
        localizacao = estado_persistente["vehicles"]["car_2"]["location"]
        ja_inside = bool(estado_persistente["vehicles"]["car_2"]["inside"])

    if localizacao != "parking" or ja_inside:
        return False

    if contador_lugar_2_ocupado < CONFIRMACOES_SENSOR:
        return False

    with estado_lock:
        estado_persistente["vehicles"]["car_2"]["inside"] = True
        estado_persistente["vehicles"]["car_2"]["location"] = "place_2"
        estado_persistente["vehicles"]["car_2"]["slot"] = 3
        estado_persistente["parking"]["place_2"]["occupied"] = True
        estado_persistente["parking"]["place_2"]["vehicle"] = "car_2"
        estado_persistente["parking"]["place_2"]["reserved_by"] = None

    guardar_estado()

    registar_evento_historico(
        "PARKING_COMPLETED",
        veiculo="car_2",
        detalhes={
            "place": "place_2",
            "background": True,
            "start_pending_or_active": True
        }
    )

    enviar_evento(
        ID_CARRO_2,
        "PARKING_COMPLETED",
        {"PLACE": "PLACE_2"}
    )
    encaminhar_v2v(
        ID_CARRO_2,
        ID_ROBOT,
        "CAR_2_PARKED",
        {"PLACE": "PLACE_2"}
    )

    with estado_lock:
        localizacao_robot = estado_persistente["vehicles"]["robot"]["location"]

    # CAR_2 has completed its movement. If the ROBOT is STOPPED at the
    # entrance after START, release the traffic reservation: CAR_2's route
    # is now clear, although the ENTRANCE SENSOR remains occupied
    # by the ROBOT awaiting its RFID. Do not send F to the ROBOT.
    # If it is still moving toward the entrance, retain the block
    # until arrival and STOP are confirmed.
    if (localizacao_robot == "entrance"
            and estado_sistema == AGUARDAR_RFID_ENTRADA):
        desbloquear_trafego()
        print(
            "TRAFEGO LIVRE: CARRO 2 estacionado e ROBOT parado na "
            "entrada a aguardar RFID. O sensor da entrada continua "
            "fisicamente ocupado pelo ROBOT."
        )

    print(
        "CARRO 2 ESTACIONADO: Lugar 2 confirmado ({} leituras <= {} cm). "
        "inside=True, location='place_2', place_2.occupied=True. "
        "O estado do ROBOT continua independente.".format(
            CONFIRMACOES_SENSOR, LIMITE_LUGAR_2_CM
        )
    )
    return True


def preparar_pos_saida_robot():
    """
    After the Robot's exit RFID identification:
    - if Car 2 was waiting in the safe spot, first wait for
      the Robot to physically leave the entrance; then ask the operator
      to move Car 2 manually back to the entrance;
    - otherwise, retain normal behavior.
    """
    if veiculo_no_safe_place("car_2"):
        bloquear_trafego(
            "AGUARDAR_ROBOT_LIBERTAR_ENTRADA_PARA_RETORNO_CARRO_2"
        )
        print(
            "Saída do ROBOT confirmada. "
            "O CARRO 2 permanece no safe place até a entrada ficar livre."
        )
        mudar_estado(
            AGUARDAR_ENTRADA_LIVRE_PARA_RETORNO_CARRO_2
        )
        return

    desbloquear_trafego()
    mudar_estado(AGUARDAR_ENTRADA_LIVRE_APOS_SAIDA)


def preparar_pos_saida_carro_2():
    """
    After Car 2's exit RFID identification:
    - if the Robot was in the safe spot, first wait for Car 2
      to physically leave the entrance;
    - only then send B to make the Robot execute the return maneuver.
    """
    if veiculo_no_safe_place("robot"):
        bloquear_trafego(
            "AGUARDAR_CARRO_2_LIBERTAR_ENTRADA_PARA_RETORNO_ROBOT"
        )
        print(
            "Saída do CARRO 2 confirmada. "
            "O ROBOT aguarda no safe place até a entrada ficar livre."
        )
        mudar_estado(
            AGUARDAR_ENTRADA_LIVRE_PARA_RETORNO_ROBOT
        )
        return

    desbloquear_trafego()
    mudar_estado(AGUARDAR_ENTRADA_LIVRE_APOS_SAIDA)


def processar_rfid(uid, texto, distancias):
    veiculo = identificar_veiculo(uid)
    print("\n" + "-" * 72)
    print("RFID recebido — UID: {}".format(uid))
    print("Estado atual: {}".format(estado_sistema))
    print("Veículo: {}".format(veiculo if veiculo else "DESCONHECIDO"))
    print("-" * 72)
    if veiculo is None:
        registar_rfid(uid, None, "denied_unknown")
        return

    # -----------------------------------------------------
    # DYNAMIC CONFLICT: a new vehicle is detected at the ENTRANCE
    # while the ROBOT is already exiting space 1.
    # -----------------------------------------------------
    if estado_sistema == AGUARDAR_RFID_CARRO_2_DURANTE_SAIDA_ROBOT:
        if not saida_robot_pausa_confirmada:
            print("Aguarde: STOP/PAUSE do ROBOT ainda sem ACK do ESP32.")
            return
        if veiculo != "car_2":
            registar_rfid(uid, veiculo, "denied_wrong_id_exit_interruption")
            print("RFID incorreto: é necessário identificar o CARRO 2.")
            return
        if not esta_ocupado(distancias["entrada"], LIMITE_ENTRADA_CM):
            registar_rfid(uid, veiculo, "denied_entrance_not_occupied")
            print("RFID negado: entrada não confirma CARRO 2 (<=15 cm).")
            return
        if estado_persistente["vehicles"]["car_2"]["inside"]:
            registar_rfid(uid, veiculo, "denied_car2_already_inside")
            print("CARRO 2 já registado dentro; verifique o veículo na entrada.")
            return
        registar_rfid(uid, "car_2", "identified_for_safe_place_during_robot_exit")
        with estado_lock:
            estado_persistente["vehicles"]["car_2"]["inside"] = False
            estado_persistente["vehicles"]["car_2"]["location"] = "moving_to_safe_place"
        guardar_estado()
        enviar_evento(
            ID_CARRO_2, "MOVE_TO_SAFE_PLACE",
            {"REASON": "ROBOT_EXIT_INTERRUPTED", "WAIT_UNTIL": "ROBOT_EXITED"}
        )
        print("CARRO 2 identificado: mova-o MANUALMENTE para o SAFE PLACE.")
        print("ROBOT permanece parado na fase {} até a entrada ficar livre."
              .format(saida_robot_estado_interrompido))
        mudar_estado(AGUARDAR_ENTRADA_LIVRE_PARA_RETOMAR_SAIDA_ROBOT)
        return

    # -----------------------------------------------------
    # SAIR1 pending: Car 2 is blocking the entrance.
    # Car 2 must first be identified by RFID.
    # Only then request manual movement to the safe spot.
    # -----------------------------------------------------
    if estado_sistema == AGUARDAR_RFID_BLOQUEADOR_PARA_SAIR1:
        if veiculo != "car_2":
            registar_rfid(
                uid,
                veiculo,
                "denied_wrong_blocker_for_sair1"
            )
            print(
                "RFID incorreto: para prosseguir o SAIR1, "
                "apresente o cartão do CARRO 2 que está na entrada."
            )
            return

        if not esta_ocupado(
            distancias["entrada"],
            LIMITE_ENTRADA_CM
        ):
            print(
                "O sensor já não deteta um veículo na entrada. "
                "Coloque novamente o CARRO 2 na entrada e apresente o RFID "
                "antes de o mover para o safe place."
            )
            return

        registar_rfid(
            uid,
            "car_2",
            "identified_for_safe_place_before_sair1"
        )

        print(
            "CARRO 2 identificado por RFID. "
            "Mova agora manualmente o CARRO 2 para o SAFE PLACE."
        )

        with estado_lock:
            estado_persistente["vehicles"]["car_2"]["location"] = (
                "moving_to_safe_place"
            )
        guardar_estado()

        enviar_evento(
            ID_CARRO_2,
            "MOVE_TO_SAFE_PLACE",
            {
                "REASON": "SAIR1_REQUESTED",
                "ROBOT_LOCATION": "PLACE_1"
            }
        )
        encaminhar_v2v(
            ID_ROBOT,
            ID_CARRO_2,
            "CLEAR_ENTRANCE_REQUIRED",
            {"REASON": "SAIR1_REQUESTED"}
        )

        mudar_estado(AGUARDAR_ENTRADA_LIVRE_PARA_SAIR1)
        return

    # -----------------------------------------------------
    # SAIR2 pending: the Robot is blocking the entrance.
    # The Robot must first be identified by RFID.
    # Only then send maneuver P automatically.
    # -----------------------------------------------------
    if estado_sistema == AGUARDAR_RFID_BLOQUEADOR_PARA_SAIR2:
        if veiculo != "robot":
            registar_rfid(
                uid,
                veiculo,
                "denied_wrong_blocker_for_sair2"
            )
            print(
                "RFID incorreto: para prosseguir o SAIR2, "
                "apresente o chip do ROBOT que está na entrada."
            )
            return

        if not esta_ocupado(
            distancias["entrada"],
            LIMITE_ENTRADA_CM
        ):
            print(
                "O sensor já não deteta um veículo na entrada. "
                "Coloque novamente o ROBOT na entrada e apresente o RFID "
                "antes da manobra para o safe place."
            )
            return

        if not cliente_ligado(ID_ROBOT):
            registar_rfid(
                uid,
                "robot",
                "denied_robot_offline_for_safe_place"
            )
            print(
                "ROBOT identificado, mas o ESP32 está desligado. "
                "Não é possível executar automaticamente a manobra P."
            )
            return

        registar_rfid(
            uid,
            "robot",
            "identified_for_safe_place_before_sair2"
        )

        print(
            "ROBOT identificado por RFID. "
            "A ordenar deslocação automática para o SAFE PLACE."
        )

        enviar_evento(
            ID_CARRO_2,
            "WAIT_FOR_ROBOT_SAFE_PLACE",
            {"REASON": "SAIR2_REQUESTED"}
        )
        encaminhar_v2v(
            ID_CARRO_2,
            ID_ROBOT,
            "CLEAR_PATH_REQUIRED",
            {"REASON": "SAIR2_REQUESTED"}
        )

        if enviar_comando_robot("P"):
            with estado_lock:
                estado_persistente["vehicles"]["robot"]["location"] = (
                    "moving_to_safe_place"
                )
            guardar_estado()
            mudar_estado(AGUARDAR_ENTRADA_LIVRE_PARA_SAIR2)
        else:
            print(
                "Falha ao enviar P ao ROBOT. "
                "O SAIR2 continua bloqueado."
            )
        return

    # -----------------------------------------------------
    # START pending: a vehicle occupies the entrance.
    # The system identifies it as CAR_2 only after validating its RFID.
    # After identification, CAR_2 is authorized to park in space 2.
    # The ROBOT remains stopped outside until the entrance is clear.
    # -----------------------------------------------------
    if estado_sistema == AGUARDAR_RFID_CARRO_2_PARA_START:
        if veiculo != "car_2":
            registar_rfid(
                uid,
                veiculo,
                "denied_start_waiting_for_car_2"
            )
            print(
                "RFID recusado: o START está a aguardar especificamente "
                "o cartão do CARRO 2 que ocupa a entrada."
            )
            return

        if not esta_ocupado(
            distancias["entrada"],
            LIMITE_ENTRADA_CM
        ):
            registar_rfid(
                uid,
                "car_2",
                "denied_car_2_not_confirmed_at_entrance_for_start"
            )
            print(
                "CARRO 2 identificado, mas o sensor já não confirma "
                "a sua presença na entrada. Volte a colocá-lo na entrada "
                "antes de apresentar o RFID."
            )
            return

        if estado_persistente["vehicles"]["car_2"]["inside"]:
            registar_rfid(
                uid,
                "car_2",
                "denied_car_2_already_inside_for_start"
            )
            print(
                "RFID recusado: o estado interno já indica que o CARRO 2 "
                "está dentro do parque."
            )
            return

        if not esta_livre(
            distancias["lugar_2"],
            LIMITE_LUGAR_2_CM
        ):
            registar_rfid(
                uid,
                "car_2",
                "denied_place_2_occupied_for_start"
            )
            print(
                "RFID recusado: o Lugar 2 não está livre para o CARRO 2."
            )
            return

        registar_rfid(
            uid,
            "car_2",
            "entry_authorized_before_pending_start"
        )

        bloquear_trafego(
            "CARRO_2_A_ESTACIONAR_ANTES_DE_RETOMAR_START"
        )

        with estado_lock:
            estado_persistente["vehicles"]["car_2"]["inside"] = False
            estado_persistente["vehicles"]["car_2"]["location"] = "parking"
            estado_persistente["vehicles"]["car_2"]["slot"] = 0
            estado_persistente["vehicles"]["car_2"]["place_1_passage_detected"] = False
            estado_persistente["parking"]["place_2"]["reserved_by"] = "car_2"

        guardar_estado()

        enviar_evento(
            ID_CARRO_2,
            "ENTRY_AUTHORIZED",
            {
                "PLACE": "PLACE_2",
                "REASON": "CLEAR_ENTRANCE_FOR_PENDING_START"
            }
        )
        encaminhar_v2v(
            ID_CARRO_2,
            ID_ROBOT,
            "CAR_2_ENTERING",
            {
                "PLACE": "PLACE_2",
                "START_PENDING": "YES"
            }
        )

        enviar_evento(
            ID_ROBOT,
            "WAIT_FOR_ENTRANCE_CLEAR",
            {
                "REASON": "CAR_2_PARKING",
                "START_PENDING": "YES"
            }
        )

        print(
            "CARRO 2 autorizado por RFID. "
            "Mova-o manualmente para o Lugar 2."
        )
        print(
            "O ROBOT permanece parado fora do parque. "
            "Assim que o sensor da entrada confirmar que ficou livre, "
            "o comando START será retomado automaticamente."
        )

        mudar_estado(
            AGUARDAR_ENTRADA_LIVRE_PARA_START_APOS_CARRO_2
        )
        return

    robot_na_entrada = esta_ocupado(distancias["entrada"], LIMITE_ENTRADA_CM)
    lugar_1_sem_robot = lugar_estacionamento_livre(distancias["lugar_1"], LIMITE_LUGAR_1_CM)
    if veiculo == "robot" and estado_sistema in (
        AGUARDAR_VEICULO_ENTRADA,
        ESTACIONADO_LUGAR_1,
        ROBOT_A_SAIR_ATE_ENTRADA,
        ROBOT_A_SAIR_APOS_CANTO
    ) and robot_na_entrada and lugar_1_sem_robot:
        robot_avanco_final_confirmado = False
        registar_rfid(uid, "robot", "exit")
        marcar_veiculo_fora("robot")
        guardar_estado()
        enviar_evento(ID_ROBOT, "EXIT_CONFIRMED", {"LOCATION": "OUTSIDE"})
        if not enviar_comando_robot("X"):
            print(
                "Aviso: saída RFID confirmada, mas não foi possível enviar "
                "o comando X para o avanço final do ROBOT até à partida."
            )
        encaminhar_v2v(ID_ROBOT, ID_CARRO_2, "ROBOT_EXITED")
        print("SAÍDA DO ROBOT CONFIRMADA.")
        preparar_pos_saida_robot()
        return
    if estado_sistema == AGUARDAR_RFID_SAIDA_ROBOT:
        if veiculo != "robot":
            print("RFID bloqueado: aguarda-se o chip do robot.")
            return
        robot_avanco_final_confirmado = False
        registar_rfid(uid, "robot", "exit")
        marcar_veiculo_fora("robot")
        guardar_estado()
        enviar_evento(ID_ROBOT, "EXIT_CONFIRMED", {"LOCATION": "OUTSIDE"})
        if not enviar_comando_robot("X"):
            print(
                "Aviso: saída RFID confirmada, mas não foi possível enviar "
                "o comando X para o avanço final do ROBOT até à partida."
            )
        encaminhar_v2v(ID_ROBOT, ID_CARRO_2, "ROBOT_EXITED")
        preparar_pos_saida_robot()
        return
    # During an automatically detected exit with the ROBOT at the entrance, CAR_2's
    # RFID must not complete the exit before the corner is confirmed,
    # P is executed, and CAR_2 is physically detected at the entrance/exit.
    if (veiculo == "car_2"
            and estado_sistema == CARRO_2_A_SAIR_ATE_CANTO
            and robot_parado_na_entrada_sem_rfid()):
        registar_rfid(uid, "car_2", "denied_exit_waiting_for_corner_and_safe_place")
        print(
            "RFID de saída do CARRO 2 ainda não autorizado: pare no "
            "CANTO e aguarde o ROBOT chegar ao safe place."
        )
        return

    lugar_2_sem_carro = lugar_2_livre_confirmavel(distancias["lugar_2"])
    if veiculo == "car_2" and estado_sistema in (
        AGUARDAR_VEICULO_ENTRADA,
        ESTACIONADO_LUGAR_2,
        CARRO_2_A_SAIR_ATE_CANTO
    ) and robot_na_entrada and lugar_2_sem_carro:
        registar_rfid(uid, "car_2", "exit")
        marcar_veiculo_fora("car_2")
        guardar_estado()
        enviar_evento(ID_CARRO_2, "EXIT_CONFIRMED", {"LOCATION": "OUTSIDE"})
        encaminhar_v2v(ID_CARRO_2, ID_ROBOT, "CAR_2_EXITED")
        preparar_pos_saida_carro_2()
        return
    if estado_sistema == AGUARDAR_RFID_SAIDA_CARRO_2:
        if veiculo != "car_2":
            print("RFID bloqueado: aguarda-se o cartão do Carro 2.")
            return
        registar_rfid(uid, "car_2", "exit")
        marcar_veiculo_fora("car_2")
        guardar_estado()
        enviar_evento(ID_CARRO_2, "EXIT_CONFIRMED", {"LOCATION": "OUTSIDE"})
        encaminhar_v2v(ID_CARRO_2, ID_ROBOT, "CAR_2_EXITED")
        preparar_pos_saida_carro_2()
        return
    if estado_sistema != AGUARDAR_RFID_ENTRADA:
        print("RFID ignorado: o sistema não está num estado de leitura.")
        return

    # The traffic reservation may be free while the ROBOT physically occupies
    # the entrance. In that situation, only the ROBOT itself may present
    # RFID to enter; the 'LIVRE' state does not mean the entrance is unoccupied.
    if robot_parado_na_entrada_sem_rfid() and veiculo != "robot":
        registar_rfid(uid, veiculo, "denied_robot_waiting_at_entrance")
        print(
            "RFID recusado: o ROBOT está fisicamente na entrada "
            "a aguardar o próprio RFID."
        )
        return

    lugar_1_livre = esta_livre(distancias["lugar_1"], LIMITE_LUGAR_1_CM)
    lugar_2_livre = lugar_2_livre_confirmavel(distancias["lugar_2"])
    if veiculo == "robot":
        # CAR_2 may still be parking in space 2 (SLOT 3).
        # Do not block solely because final space 2 confirmation is missing if it has
        # passed the space 1 sensor AND both route sensors indicate a clear path.
        # This permits concurrent parking operations in separate slots.
        if not corredor_robot_livre_para_rfid(distancias):
            registar_rfid(uid, "robot", "denied_car_2_on_shared_slots")
            print(
                "RFID ROBOT: aguarde. O CARRO 2 ainda ocupa o corredor "
                "partilhado (Slots 1/2) ou a passagem do Lugar 1 não "
                "foi confirmada. Só poderá avançar com CARRO 2 no Slot 3 "
                "e sensores Canto/Lugar 1 livres."
            )
            return
        if estado_persistente["vehicles"]["robot"]["inside"] and lugar_1_livre:
            marcar_veiculo_fora("robot")
            guardar_estado()
        if estado_persistente["vehicles"]["robot"]["inside"]:
            registar_rfid(uid, "robot", "denied_already_inside")
            return
        if not lugar_1_livre:
            registar_rfid(uid, "robot", "denied_place_1_occupied")
            return
        if not cliente_ligado(ID_ROBOT):
            registar_rfid(uid, "robot", "denied_robot_offline")
            print("Entrada recusada: ESP32 ROBOT desligado.")
            return
        bloquear_trafego("ROBOT_A_ESTACIONAR")
        registar_rfid(uid, "robot", "entry_authorized")
        with estado_lock:
            estado_persistente["vehicles"]["robot"]["location"] = "entering"
            estado_persistente["vehicles"]["robot"]["slot"] = 1
        guardar_estado()
        enviar_evento(ID_ROBOT, "ENTRY_AUTHORIZED", {"PLACE": "PLACE_1"})
        encaminhar_v2v(ID_ROBOT, ID_CARRO_2, "ROBOT_ENTERING", {"PLACE": "PLACE_1"})
        if enviar_comando_robot("F"):
            mudar_estado(ROBOT_AVANCAR_ATE_CANTO)
        return
    if veiculo == "car_2":
        if estado_persistente["vehicles"]["car_2"]["inside"] and lugar_2_livre:
            marcar_veiculo_fora("car_2")
            guardar_estado()
        if estado_persistente["vehicles"]["car_2"]["inside"]:
            registar_rfid(uid, "car_2", "denied_already_inside")
            return
        if not lugar_2_livre:
            registar_rfid(uid, "car_2", "denied_place_2_occupied")
            return
        bloquear_trafego("CARRO_2_A_ESTACIONAR")
        registar_rfid(uid, "car_2", "entry_authorized")
        with estado_lock:
            estado_persistente["vehicles"]["car_2"]["location"] = "parking"
            estado_persistente["vehicles"]["car_2"]["slot"] = 0
            estado_persistente["vehicles"]["car_2"]["place_1_passage_detected"] = False
            estado_persistente["parking"]["place_2"]["reserved_by"] = "car_2"
        guardar_estado()
        enviar_evento(ID_CARRO_2, "ENTRY_AUTHORIZED", {"PLACE": "PLACE_2"})
        encaminhar_v2v(ID_CARRO_2, ID_ROBOT, "CAR_2_ENTERING", {"PLACE": "PLACE_2"})
        mudar_estado(CARRO_2_A_ESTACIONAR)


def robot_comecou_saida():
    return estado_persistente["vehicles"]["robot"]["inside"] and estado_persistente["parking"]["place_1"]["occupied"] and contador_lugar_1_livre >= CONFIRMACOES_SENSOR


def carro_2_comecou_saida():
    return estado_persistente["vehicles"]["car_2"]["inside"] and estado_persistente["parking"]["place_2"]["occupied"] and contador_lugar_2_livre >= CONFIRMACOES_SENSOR


def verificar_percurso_saida_robot_livre(distancias):
    """
    Validate all conditions required for SAIR1.

    Car 2 may be parked in space 2; this does not obstruct
    the Robot's exit route.

    The entrance may be:
      - clear: start the exit immediately;
      - occupied: assume Car 2 is blocking it and must
        be moved manually to the safe spot BEFORE the Robot
        leaves space 1;
      - unreadable: refuse the exit for safety.
    """
    problemas = []

    if not sensores_inicializados:
        problemas.append("sensores ainda não inicializados")

    if not cliente_ligado(ID_ROBOT):
        problemas.append("ESP32 do ROBOT não está ligado")

    lugar_1 = distancias["lugar_1"]
    lugar_2 = distancias["lugar_2"]
    canto = distancias["canto"]
    entrada = distancias["entrada"]

    if not esta_ocupado(lugar_1, LIMITE_LUGAR_1_CM):
        problemas.append(
            "Lugar 1 não confirma a presença do ROBOT (distância={})".format(
                texto_distancia(lugar_1)
            )
        )

    # The state of space 2 does NOT block SAIR1.
    # Car 2 may remain parked in space 2.
    # SAIR1 authorization depends on:
    # - Robot confirmed in space 1;
    # - a clear corner;
    # - a valid entrance sensor reading.
    # If the entrance is occupied, the safe-spot logic handles the conflict.

    if not esta_livre(canto, LIMITE_CANTO_SAIDA_CM):
        problemas.append(
            "canto bloqueado ou sem leitura válida (distância={})".format(
                texto_distancia(canto)
            )
        )

    if entrada is None:
        problemas.append("sensor da entrada sem leitura válida")

    with estado_lock:
        robot_inside = bool(
            estado_persistente["vehicles"]["robot"]["inside"]
        )
        lugar_1_json = bool(
            estado_persistente["parking"]["place_1"]["occupied"]
        )

    if not robot_inside or not lugar_1_json:
        problemas.append(
            "estado interno não confirma o ROBOT estacionado no Lugar 1"
        )

    entrada_ocupada = esta_ocupado(entrada, LIMITE_ENTRADA_CM)
    return len(problemas) == 0, problemas, entrada_ocupada


def executar_sair1_com_entrada_livre():
    """Start physical SAIR1 movement after the entrance is confirmed clear."""
    print("\n" + "=" * 72)
    print("SAIR1 AUTORIZADO — PERCURSO LIVRE")
    print(
        "O ROBOT permanece no Lugar 1 até este momento. "
        "A iniciar rotação de saída e percurso até ao canto."
    )
    print("=" * 72)

    bloquear_trafego("ROBOT_A_SAIR_COMANDO")

    with estado_lock:
        estado_persistente["vehicles"]["robot"]["location"] = (
            "turning_180_from_place_1"
        )
        estado_persistente["parking"]["place_1"]["occupied"] = False
        estado_persistente["parking"]["place_1"]["vehicle"] = None
    guardar_estado()

    enviar_evento(
        ID_ROBOT,
        "EXIT_STARTED",
        {
            "FROM": "PLACE_1",
            "MODE": "MANUAL_COMMAND",
            "COMMAND": "SAIR1"
        }
    )
    encaminhar_v2v(
        ID_ROBOT,
        ID_CARRO_2,
        "ROBOT_EXITING",
        {"MODE": "SAIR1"}
    )

    if enviar_comando_robot("U"):
        mudar_estado(ROBOT_SAIDA_COMANDO_ATE_CANTO)
        return True

    print("SAIR1 falhou: não foi possível enviar o comando U ao ROBOT.")

    with estado_lock:
        estado_persistente["vehicles"]["robot"]["location"] = "place_1"
        estado_persistente["parking"]["place_1"]["occupied"] = True
        estado_persistente["parking"]["place_1"]["vehicle"] = "robot"
    desbloquear_trafego()
    guardar_estado()
    return False


def iniciar_saida_robot_por_comando(distancias):
    registar_evento_historico(
        "COMMAND_REQUESTED",
        veiculo="robot",
        detalhes={"command": "sair1"}
    )

    if estado_sistema not in (
        AGUARDAR_VEICULO_ENTRADA,
        AGUARDAR_RFID_ENTRADA,
        ESTACIONADO_LUGAR_1
    ):
        print(
            "SAIR1 recusado: estado atual '{}' não permite iniciar "
            "uma nova saída.".format(estado_sistema)
        )
        return False

    if estado_persistente["traffic"]["blocked"]:
        print(
            "SAIR1 recusado: tráfego bloqueado ({})".format(
                estado_persistente["traffic"]["reason"]
            )
        )
        return False

    json_ok, problemas_json = validar_estacionamento_json("robot")
    if not json_ok:
        print(
            "SAIR1 NÃO AUTORIZADO — parking_state.json "
            "não confirma o ROBOT estacionado no Lugar 1:"
        )
        for problema in problemas_json:
            print("  - {}".format(problema))
        return False

    print(
        "SAIR1: parking_state.json confirma o ROBOT estacionado no Lugar 1."
    )

    condicoes_ok, problemas, entrada_ocupada = (
        verificar_percurso_saida_robot_livre(distancias)
    )

    if not condicoes_ok:
        print("\nSAIR1 NÃO AUTORIZADO:")
        for problema in problemas:
            print("  - {}".format(problema))
        return False

    if entrada_ocupada:
        print("\n" + "=" * 72)
        print("SAIR1 A AGUARDAR — EXISTE UM VEÍCULO NA ENTRADA")
        print(
            "O ROBOT permanece estacionado no Lugar 1 e NÃO inicia a saída."
        )
        print(
            "Antes de mover o veículo da entrada, apresente o cartão RFID "
            "do CARRO 2."
        )
        print(
            "Depois da identificação, será pedido que mova o CARRO 2 "
            "manualmente para o SAFE PLACE."
        )
        print(
            "Quando a entrada ficar livre, o SAIR1 prossegue automaticamente."
        )
        print("=" * 72)

        bloquear_trafego(
            "AGUARDAR_RFID_CARRO_2_ANTES_DO_SAIR1"
        )

        enviar_evento(
            ID_CARRO_2,
            "RFID_SAFE_PLACE_REQUIRED",
            {"REASON": "SAIR1_REQUESTED"}
        )

        mudar_estado(AGUARDAR_RFID_BLOQUEADOR_PARA_SAIR1)
        return True

    return executar_sair1_com_entrada_livre()


def verificar_percurso_saida_carro_2_livre(distancias):
    """
    Validate SAIR2.

    The Robot may be parked in space 1; this does not obstruct
    Car 2's exit route.

    If the entrance is occupied, do not start the exit: in this case,
    the safe-spot logic handles the obstruction before Car 2
    physically leaves space 2.
    """
    problemas = []

    if not sensores_inicializados:
        problemas.append("sensores ainda não inicializados")

    if not cliente_ligado(ID_CARRO_2):
        problemas.append("ESP32 do CARRO 2 não está ligado")

    lugar_1 = distancias["lugar_1"]
    lugar_2 = distancias["lugar_2"]
    canto = distancias["canto"]
    entrada = distancias["entrada"]

    if not esta_ocupado(lugar_2, LIMITE_LUGAR_2_CM):
        problemas.append(
            "Lugar 2 não confirma a presença do CARRO 2 (distância={})".format(
                texto_distancia(lugar_2)
            )
        )

    # The state of space 1 does NOT block SAIR2.
    # The Robot may remain parked in space 1.
    # SAIR2 authorization depends on:
    # - Car 2 confirmed in space 2;
    # - a clear corner;
    # - a valid entrance sensor reading.
    # If the entrance is occupied, the safe-spot logic handles the conflict.

    if not esta_livre(canto, LIMITE_CANTO_SAIDA_CM):
        problemas.append(
            "canto bloqueado ou sem leitura válida (distância={})".format(
                texto_distancia(canto)
            )
        )

    if entrada is None:
        problemas.append("sensor da entrada sem leitura válida")

    with estado_lock:
        carro_inside = bool(
            estado_persistente["vehicles"]["car_2"]["inside"]
        )
        lugar_2_json = bool(
            estado_persistente["parking"]["place_2"]["occupied"]
        )

    if not carro_inside or not lugar_2_json:
        problemas.append(
            "estado interno não confirma o CARRO 2 estacionado no Lugar 2"
        )

    entrada_ocupada = esta_ocupado(entrada, LIMITE_ENTRADA_CM)
    return len(problemas) == 0, problemas, entrada_ocupada


def executar_sair2_com_entrada_livre():
    """Authorize Car 2's manual movement after the entrance is clear."""
    print("\n" + "=" * 72)
    print("SAIR2 AUTORIZADO — PERCURSO LIVRE")
    print(
        "O CARRO 2 pode agora abandonar o Lugar 2. "
        "Mova-o manualmente até ao CANTO."
    )
    print("=" * 72)

    bloquear_trafego("CARRO_2_A_SAIR_COMANDO")

    with estado_lock:
        estado_persistente["vehicles"]["car_2"]["location"] = (
            "moving_from_place_2_to_corner"
        )
        # Space 2 remains marked as occupied until its sensor physically
        # confirms that Car 2 has left the space.
    guardar_estado()

    enviar_evento(
        ID_CARRO_2,
        "MANUAL_EXIT_TO_CORNER",
        {
            "FROM": "PLACE_2",
            "NEXT": "CORNER",
            "COMMAND": "SAIR2"
        }
    )
    encaminhar_v2v(
        ID_CARRO_2,
        ID_ROBOT,
        "CAR_2_EXITING",
        {"MODE": "SAIR2"}
    )

    mudar_estado(CARRO_2_SAIDA_COMANDO_ATE_CANTO)
    return True


def iniciar_saida_carro_2_por_comando(distancias):
    registar_evento_historico(
        "COMMAND_REQUESTED",
        veiculo="car_2",
        detalhes={"command": "sair2"}
    )

    if estado_sistema not in (
        AGUARDAR_VEICULO_ENTRADA,
        AGUARDAR_RFID_ENTRADA,
        ESTACIONADO_LUGAR_2
    ):
        print(
            "SAIR2 recusado: estado atual '{}' não permite iniciar "
            "uma nova saída.".format(estado_sistema)
        )
        return False

    # The ROBOT has already been identified during START and is stopped at the entrance.
    # In this case, if CAR_2 is confirmed in space 2, allow
    # SAIR2 without requiring the ROBOT's RFID a second time. Maneuver P
    # occurs only when CAR_2 is detected at the CORNER (three readings).
    if (estado_sistema == AGUARDAR_RFID_ENTRADA
            and robot_parado_na_entrada_sem_rfid()):
        json_ok, problemas_json = validar_estacionamento_json("car_2")
        condicoes_ok, problemas, entrada_ocupada = (
            verificar_percurso_saida_carro_2_livre(distancias)
        )
        if not json_ok or not condicoes_ok or not entrada_ocupada:
            print("SAIR2 bloqueado: cenário START com ROBOT na entrada.")
            for problema in problemas_json + problemas:
                print("  - {}".format(problema))
            if not entrada_ocupada:
                print("  - Entrada não confirma fisicamente o ROBOT.")
            return False
        if not cliente_ligado(ID_ROBOT):
            print("SAIR2 bloqueado: ESP32 do ROBOT desligado (safe place indisponível).")
            return False

        print("\n" + "=" * 72)
        print("SAIR2 autorizado com ROBOT na entrada após START.")
        print("Mova CARRO 2 até ao CANTO e PARE. Só então enviaremos P ao ROBOT.")
        print("Não continue do canto até o ROBOT libertar a entrada.")
        print("=" * 72)
        enviar_evento(
            ID_CARRO_2, "MANUAL_EXIT_TO_CORNER",
            {"WAIT_AT_CORNER": "YES", "REASON": "ROBOT_AT_ENTRANCE"}
        )
        iniciar_saida_carro_2()
        return True

    if estado_persistente["traffic"]["blocked"]:
        print(
            "SAIR2 recusado: tráfego bloqueado ({})".format(
                estado_persistente["traffic"]["reason"]
            )
        )
        return False

    json_ok, problemas_json = validar_estacionamento_json("car_2")
    if not json_ok:
        print(
            "SAIR2 NÃO AUTORIZADO — parking_state.json "
            "não confirma o CARRO 2 estacionado no Lugar 2:"
        )
        for problema in problemas_json:
            print("  - {}".format(problema))
        return False

    print(
        "SAIR2: parking_state.json confirma o CARRO 2 estacionado no Lugar 2."
    )

    condicoes_ok, problemas, entrada_ocupada = (
        verificar_percurso_saida_carro_2_livre(distancias)
    )

    if not condicoes_ok:
        print("\nSAIR2 NÃO AUTORIZADO:")
        for problema in problemas:
            print("  - {}".format(problema))
        return False

    if entrada_ocupada:
        print("\n" + "=" * 72)
        print("SAIR2 A AGUARDAR — EXISTE UM VEÍCULO NA ENTRADA")
        print(
            "O CARRO 2 permanece estacionado no Lugar 2 e NÃO inicia a saída."
        )
        print(
            "Antes de mover o veículo da entrada, apresente o chip RFID "
            "do ROBOT."
        )
        print(
            "Depois da identificação, o parque ordena automaticamente "
            "a manobra P para o SAFE PLACE."
        )
        print(
            "Quando a entrada ficar livre, o SAIR2 prossegue automaticamente."
        )
        print("=" * 72)

        bloquear_trafego(
            "AGUARDAR_RFID_ROBOT_ANTES_DO_SAIR2"
        )

        enviar_evento(
            ID_CARRO_2,
            "WAIT_FOR_ROBOT_RFID",
            {"REASON": "SAIR2_REQUESTED"}
        )

        mudar_estado(AGUARDAR_RFID_BLOQUEADOR_PARA_SAIR2)
        return True

    return executar_sair2_com_entrada_livre()


def iniciar_robot_da_partida_ate_entrada(distancias):
    """
    START scenario.

    Case A - entrance clear:
      - the ROBOT is outside the parking area;
      - send F;
      - advance toward the entrance sensor;
      - first normal-cycle reading + second reading 25 ms later;
      - STOP and wait for normal RFID identification.

    Case B - entrance occupied, CAR_2 RFID not yet presented:
      - the ROBOT remains stopped outside and START stays pending;
      - wait for CAR_2 RFID identification and a physically clear entrance;
      - automatically resume movement to the entrance.

    Case C - CAR_2 is already parking (RFID presented):
      - keep START pending without requesting a second RFID identification;
      - automatically resume START when the entrance is clear.
    """
    # Normal entrance detection moves the state machine from
    # AGUARDAR_VEICULO_ENTRADA to AGUARDAR_RFID_ENTRADA BEFORE reading
    # keyboard commands. START must also be accepted in that state.
    # Also accept the command during CARRO_2_A_ESTACIONAR if CAR_2's
    # card was already presented before START was entered.
    estados_inicio_start = (
        AGUARDAR_VEICULO_ENTRADA,
        AGUARDAR_RFID_ENTRADA,
        CARRO_2_A_ESTACIONAR
    )
    if estado_sistema not in estados_inicio_start:
        print(
            "START recusado: estado atual '{}' não permite iniciar "
            "a aproximação à entrada.".format(estado_sistema)
        )
        return False

    # Traffic is normally already blocked while CAR_2 is parking.
    # In this case, do not cancel START: keep it pending until
    # the entrance is physically clear. Reject other blocking conditions.
    trafego = estado_persistente["traffic"]
    if trafego["blocked"] and not (
        estado_sistema == CARRO_2_A_ESTACIONAR
        and trafego["reason"] == "CARRO_2_A_ESTACIONAR"
    ):
        print(
            "START recusado: tráfego bloqueado ({})".format(
                trafego["reason"]
            )
        )
        return False

    if not cliente_ligado(ID_ROBOT):
        print(
            "START recusado: ESP32 do ROBOT não está ligado."
        )
        return False

    entrada = distancias["entrada"]

    if entrada is None:
        print(
            "START recusado: sensor da entrada sem leitura válida."
        )
        return False

    with estado_lock:
        robot_inside = bool(
            estado_persistente["vehicles"]["robot"]["inside"]
        )
        robot_location = estado_persistente["vehicles"]["robot"]["location"]

    if robot_inside:
        print(
            "START recusado: parking_state.json indica que o ROBOT "
            "já está dentro do parque."
        )
        return False

    if robot_location != "outside":
        print(
            "START recusado: parking_state.json indica location='{}', "
            "mas para este cenário é esperado 'outside'.".format(
                robot_location
            )
        )
        return False

    registar_evento_historico(
        "COMMAND_REQUESTED",
        veiculo="robot",
        detalhes={"command": "start"}
    )

    # CAR_2 has already presented its RFID and is parking.
    # Do not request RFID again or ignore START.
    # Store the continuation in the state machine and monitor
    # space 2 in parallel while the ROBOT waits or approaches.
    if estado_sistema == CARRO_2_A_ESTACIONAR:
        with estado_lock:
            localizacao_carro_2 = estado_persistente["vehicles"]["car_2"]["location"]
        if localizacao_carro_2 != "parking":
            print(
                "START recusado: CARRO 2 está no estado de estacionamento "
                "mas a localização persistente é '{}' (esperado 'parking').".format(
                    localizacao_carro_2
                )
            )
            return False

        bloquear_trafego("CARRO_2_A_ESTACIONAR_ANTES_DE_RETOMAR_START")
        enviar_evento(
            ID_ROBOT,
            "WAIT_FOR_ENTRANCE_CLEAR",
            {"REASON": "CAR_2_PARKING", "START_PENDING": "YES"}
        )
        print(
            "START registado enquanto o CARRO 2 já está a estacionar. "
            "ROBOT permanece parado na partida até a entrada estar livre."
        )
        mudar_estado(AGUARDAR_ENTRADA_LIVRE_PARA_START_APOS_CARRO_2)
        return True

    # If the sensor has just become clear, the normal
    # AGUARDAR_RFID_ENTRADA state may not yet have been cleared by the counters.
    # Do not start the Robot from that temporary state.
    if estado_sistema == AGUARDAR_RFID_ENTRADA and not esta_ocupado(
        entrada, LIMITE_ENTRADA_CM
    ):
        print(
            "START: a entrada já parece livre, mas ainda está em "
            "AGUARDAR_RFID_ENTRADA. Aguarde a confirmação de entrada livre "
            "e volte a executar START."
        )
        return False

    # -----------------------------------------------------
    # Entrance occupied: store START as pending.
    # The Robot does NOT move until CAR_2 is identified by RFID
    # and physically leaves the entrance to park.
    # -----------------------------------------------------
    if esta_ocupado(entrada, LIMITE_ENTRADA_CM):
        bloquear_trafego(
            "START_PENDENTE_AGUARDA_RFID_CARRO_2_NA_ENTRADA"
        )

        print("\n" + "=" * 72)
        print("START PENDENTE — ENTRADA OCUPADA")
        print(
            "O ROBOT permanece parado fora do parque."
        )
        print(
            "Apresente o RFID do CARRO 2 que se encontra na entrada."
        )
        print(
            "Depois da autorização, mova o CARRO 2 para o Lugar 2. "
            "Assim que o sensor da entrada ficar livre, o ROBOT "
            "retoma automaticamente o START."
        )
        print("=" * 72)

        enviar_evento(
            ID_ROBOT,
            "START_WAITING_FOR_CAR_2_RFID",
            {
                "REASON": "ENTRANCE_OCCUPIED",
                "LOCATION": "OUTSIDE"
            }
        )

        enviar_evento(
            ID_CARRO_2,
            "RFID_ENTRY_REQUIRED",
            {
                "PLACE": "PLACE_2",
                "REASON": "PENDING_ROBOT_START"
            }
        )

        mudar_estado(AGUARDAR_RFID_CARRO_2_PARA_START)
        return True

    # -----------------------------------------------------
    # Entrance clear: retain the existing START behavior.
    # -----------------------------------------------------
    print("\n" + "=" * 72)
    print("START AUTORIZADO")
    print(
        "ROBOT fora do parque. A avançar em frente até ao sensor da entrada."
    )
    print(
        "Na entrada serão usadas 2 leituras: "
        "1 do ciclo normal + 1 confirmação 25 ms depois."
    )
    print("=" * 72)

    with estado_lock:
        estado_persistente["vehicles"]["robot"]["location"] = (
            "approaching_entrance"
        )
        estado_persistente["vehicles"]["robot"]["slot"] = 0
    guardar_estado()

    enviar_evento(
        ID_ROBOT,
        "START_TO_ENTRANCE",
        {
            "FROM": "OUTSIDE",
            "DESTINATION": "ENTRANCE"
        }
    )

    if enviar_comando_robot("F"):
        mudar_estado(ROBOT_START_ATE_ENTRADA)
        return True

    print(
        "START falhou: não foi possível enviar o comando F ao ROBOT."
    )

    with estado_lock:
        estado_persistente["vehicles"]["robot"]["location"] = "outside"
    guardar_estado()

    return False


def processar_comandos_interface(distancias):
    while True:
        try:
            comando = fila_comandos_interface.get_nowait()
        except queue.Empty:
            return

        comando = comando.strip().lower()

        if comando == "start":
            iniciar_robot_da_partida_ate_entrada(distancias)
        elif comando == "sair1":
            iniciar_saida_robot_por_comando(distancias)
        elif comando == "sair2":
            iniciar_saida_carro_2_por_comando(distancias)
        elif comando in ("ajuda", "help"):
            print("\nComandos disponíveis:")
            print(
                "  start  - ROBOT parte de fora do parque e avança até à entrada"
            )
            print("  sair1  - saída autónoma do ROBOT a partir do Lugar 1")
            print("  sair2  - saída manual do CARRO 2 a partir do Lugar 2")
            print("  ajuda   - mostrar esta ajuda")
        else:
            print(
                "Comando de interface desconhecido: '{}'. "
                "Use 'ajuda'.".format(comando)
            )


def interface_comandos_bloqueante():
    """
    Run the keyboard interface in the MAIN thread.

    Run the sensor/parking loop in a separate thread. This avoids
    input() problems in daemon threads observed in some
    Raspberry Pi terminals and IDEs.
    """
    global programa_ativo

    print("\n" + "=" * 72)
    print("INTERFACE DE COMANDOS")
    print("=" * 72)
    print("Escreva um comando e carregue ENTER:")
    print("  start  -> ROBOT parte de fora e avança até à entrada")
    print("  sair1  -> saída do ROBOT do Lugar 1")
    print("  sair2  -> saída manual do CARRO 2 do Lugar 2")
    print("  ajuda  -> mostrar comandos")
    print("  estado -> mostrar estado atual do parking_state.json")
    print("  terminar -> fechar o programa")
    print("=" * 72)

    while programa_ativo:
        try:
            comando = input("\nPARKING> ").strip().lower()
        except EOFError:
            print(
                "\nEntrada de teclado indisponível. "
                "Execute o programa num terminal interativo."
            )
            programa_ativo = False
            return

        if not comando:
            continue

        if comando in ("terminar", "exit", "quit"):
            programa_ativo = False
            return

        if comando in ("estado", "state"):
            with estado_lock:
                print("\nEstado persistente atual:")
                print(
                    json.dumps(
                        estado_persistente,
                        indent=4,
                        sort_keys=True
                    )
                )
            continue

        # Only enqueue the command. The parking loop executes it
        # using the latest sensor readings, avoiding race conditions.
        fila_comandos_interface.put(comando)

        print(
            "Comando '{}' recebido. "
            "A infraestrutura vai processá-lo.".format(comando)
        )


def iniciar_saida_robot():
    bloquear_trafego("ROBOT_A_SAIR")
    with estado_lock:
        estado_persistente["vehicles"]["robot"]["location"] = "exiting"
        estado_persistente["parking"]["place_1"]["occupied"] = False
        estado_persistente["parking"]["place_1"]["vehicle"] = None
    guardar_estado()
    enviar_evento(ID_ROBOT, "EXIT_STARTED", {"FROM": "PLACE_1"})
    encaminhar_v2v(ID_ROBOT, ID_CARRO_2, "ROBOT_EXITING")
    mudar_estado(ROBOT_A_SAIR_ATE_ENTRADA)



def regressar_robot_do_safe_place():
    """
    Command the Robot to reverse the safe-spot maneuver after
    Car 2's exit has been confirmed by RFID.
    """
    with estado_lock:
        robot_no_safe_place = (
            estado_persistente["vehicles"]["robot"]["location"]
            == "safe_place"
        )

    if not robot_no_safe_place:
        return False

    print(
        "Saída do Carro 2 confirmada. "
        "O robot vai regressar do safe place à posição inicial."
    )

    enviar_evento(
        ID_ROBOT,
        "RETURN_FROM_SAFE_PLACE",
        {"DESTINATION": "ENTRANCE"}
    )
    encaminhar_v2v(
        ID_CARRO_2,
        ID_ROBOT,
        "CAR_2_EXITED_RETURN_TO_ENTRANCE"
    )

    if enviar_comando_robot("B"):
        with estado_lock:
            estado_persistente["vehicles"]["robot"]["location"] = (
                "returning_from_safe_place"
            )
        guardar_estado()
        return True

    print(
        "Não foi possível enviar o comando de regresso ao robot."
    )
    return False

def iniciar_saida_carro_2():
    global contador_canto_saida
    # New maneuver: never inherit corner confirmations from a previous exit.
    contador_canto_saida = 0
    bloquear_trafego("CARRO_2_A_SAIR")
    with estado_lock:
        estado_persistente["vehicles"]["car_2"]["location"] = "exiting"
        estado_persistente["parking"]["place_2"]["occupied"] = False
        estado_persistente["parking"]["place_2"]["vehicle"] = None
    guardar_estado()
    enviar_evento(ID_CARRO_2, "EXIT_STARTED", {"FROM": "PLACE_2"})
    encaminhar_v2v(ID_CARRO_2, ID_ROBOT, "CAR_2_EXITING")
    mudar_estado(CARRO_2_A_SAIR_ATE_CANTO)


def robot_parado_na_entrada_sem_rfid():
    """Move the Robot to the safe spot only if identified through START.

    An occupied entrance reading alone does not identify the vehicle.
    The location='entrance' attribute was set after the ROBOT
    stopped at the entrance, as confirmed by the sensor and server.
    """
    with estado_lock:
        info = estado_persistente["vehicles"]["robot"]
        return not info["inside"] and info["location"] == "entrance"


def preparar_saida_auto_carro_2_com_robot_na_entrada(distancias):
    """At the CORNER (three readings <= 15 cm), send P to the ROBOT at the entrance.

    Do not mark the ROBOT as being in the safe spot prematurely: confirm
    the maneuver only when the entrance sensor indicates clear (three readings).
    Authorize Car 2's exit only afterward.
    """
    if not robot_parado_na_entrada_sem_rfid():
        return False

    if not esta_ocupado(distancias["entrada"], LIMITE_ENTRADA_CM):
        print(
            "SAIDA CARRO 2 EM ESPERA: Robot registado na entrada, "
            "mas o sensor não confirma a sua presença. "
            "Não é seguro iniciar a manobra."
        )
        return True  # block the exit until physical confirmation is available

    if not cliente_ligado(ID_ROBOT):
        print(
            "SAIDA CARRO 2 EM ESPERA: ESP32 do Robot desligado. "
            "Mantenha o CARRO 2 parado até libertar o percurso."
        )
        return True

    # Never advance Car 2 while the Robot occupies the entrance.
    enviar_evento(
        ID_CARRO_2,
        "WAIT_FOR_ROBOT_SAFE_PLACE",
        {"REASON": "CAR_2_AUTOMATIC_EXIT", "ACTION": "WAIT"}
    )
    encaminhar_v2v(
        ID_CARRO_2, ID_ROBOT, "CLEAR_ENTRANCE_REQUIRED",
        {"REASON": "CAR_2_AUTOMATIC_EXIT"}
    )
    if not enviar_comando_robot("P"):
        print("SAIDA CARRO 2 EM ESPERA: falhou comando P ao Robot.")
        return True

    with estado_lock:
        estado_persistente["vehicles"]["robot"]["location"] = (
            "moving_to_safe_place"
        )
    guardar_estado()
    bloquear_trafego("ROBOT_A_LIBERTAR_ENTRADA_PARA_SAIDA_AUTO_CARRO_2")
    print(
        "CARRO 2 confirmado no CANTO com o Robot na entrada. "
        "Enviado P ao Robot. Mantenha o CARRO 2 PARADO no CANTO "
        "até a infraestrutura confirmar o safe place."
    )
    mudar_estado(AGUARDAR_ROBOT_NO_SAFE_PLACE_PARA_SAIDA_AUTO_CARRO_2)
    return True


def processar_maquina_estados(distancias):
    global saida_robot_estado_interrompido
    global saida_robot_pausa_confirmada
    global saida_robot_retomada_confirmada
    global contador_canto_saida
    global contador_entrada_saida_robot
    global contador_entrada_sair1
    entrada = distancias["entrada"]
    canto = distancias["canto"]
    lugar_1 = distancias["lugar_1"]
    lugar_2 = distancias["lugar_2"]
    atualizar_contadores(distancias)
    reconciliar_json_com_sensores()
    if not sensores_inicializados:
        return
    # Explicitly track CAR_2 without changing the global state
    # used by the ROBOT. This information is useful when the sensor is
    # in the indeterminate zone (above 10 and below 15 cm): in that zone,
    # space 2 must NOT be marked as occupied.
    if (estado_sistema == AGUARDAR_RFID_ENTRADA
            and estado_persistente["vehicles"]["car_2"]["location"] == "parking"
            and contador_lugar_2_ocupado < CONFIRMACOES_SENSOR
            and pode_mostrar_mensagem()):
        print(
            "CARRO 2 A ESTACIONAR (paralelo): Lugar 2={}, "
            "leituras ocupadas={}/{} (ocupado <= {} cm). "
            "ROBOT continua a aguardar RFID na entrada.".format(
                texto_distancia(lugar_2),
                contador_lugar_2_ocupado,
                CONFIRMACOES_SENSOR,
                LIMITE_LUGAR_2_CM
            )
        )

    # SLOTS: monitor independently of the ROBOT's main state.
    atualizar_slots_carro_2_em_background(distancias)

    # CAR_2 parking confirmation is INDEPENDENT of the ROBOT's
    # main state. RFID authorized entry and set location='parking';
    # three space 2 readings validate arrival even if the Robot has since
    # reached the entrance and is waiting for its own RFID.
    # CARRO_2_A_ESTACIONAR retains its original handling without duplicate
    # notifications; in all other states, run this check independently.
    if estado_sistema != CARRO_2_A_ESTACIONAR:
        finalizar_estacionamento_carro_2_em_background()

    # Quickly check for a NEW vehicle at the entrance while the ROBOT
    # leaves space 1. Interruption preserves the ESP32's U/L/F phase.
    # During manual exit, require the operator to STOP the manual movement.
    if estado_sistema in (
        ROBOT_SAIDA_COMANDO_ATE_CANTO,
        ROBOT_A_SAIR_ATE_ENTRADA,
    ):
        if tentar_detetar_conflito_entrada_durante_saida(entrada):
            return

    estados_saida = (
        # Despite this tuple's historical name, it also includes START
        # to prevent other automatic detections from interrupting
        # the ROBOT's approach to the entrance.
        ROBOT_START_ATE_ENTRADA,
        # The ROBOT may traverse slots 1/2 while CAR_2 is still
        # parking in slot 3. Do not allow an automatic exit detector
        # to replace the main state during that movement.
        ROBOT_AVANCAR_ATE_CANTO,
        ROBOT_AVANCAR_ATE_LUGAR_1,
        AGUARDAR_RFID_CARRO_2_PARA_START,
        AGUARDAR_ENTRADA_LIVRE_PARA_START_APOS_CARRO_2,
        AGUARDAR_ROBOT_NO_SAFE_PLACE_PARA_SAIDA_AUTO_CARRO_2,
        ROBOT_A_SAIR_ATE_ENTRADA,
        ROBOT_SAIDA_COMANDO_ATE_CANTO,
        ROBOT_SAIDA_COMANDO_ATE_ENTRADA,
        AGUARDAR_RFID_CARRO_2_DURANTE_SAIDA_ROBOT,
        AGUARDAR_ENTRADA_LIVRE_PARA_RETOMAR_SAIDA_ROBOT,
        AGUARDAR_ACK_RETOMA_SAIDA_ROBOT,
        CARRO_2_SAIDA_COMANDO_ATE_CANTO,
        CARRO_2_SAIDA_COMANDO_ATE_ENTRADA,
        AGUARDAR_RFID_BLOQUEADOR_PARA_SAIR1,
        AGUARDAR_RFID_BLOQUEADOR_PARA_SAIR2,
        AGUARDAR_ENTRADA_LIVRE_PARA_SAIR1,
        AGUARDAR_ENTRADA_LIVRE_PARA_SAIR2,
        AGUARDAR_CARRO_2_NO_SAFE_PLACE,
        ROBOT_A_SAIR_APOS_CANTO,
        AGUARDAR_RFID_SAIDA_ROBOT,
        CARRO_2_A_SAIR_ATE_CANTO,
        AGUARDAR_RFID_SAIDA_CARRO_2,
        AGUARDAR_ENTRADA_LIVRE_APOS_SAIDA,
        AGUARDAR_ENTRADA_LIVRE_PARA_RETORNO_ROBOT,
        AGUARDAR_ROBOT_RETORNAR_ENTRADA,
        AGUARDAR_ENTRADA_LIVRE_PARA_RETORNO_CARRO_2,
        AGUARDAR_CARRO_2_RETORNAR_ENTRADA
    )
    if estado_sistema not in estados_saida:
        robot_saiu_lugar = robot_comecou_saida()
        carro_2_saiu_lugar = carro_2_comecou_saida()

        if robot_saiu_lugar:
            if carro_2_saiu_lugar:
                print(
                    "Os dois veículos iniciaram a saída. "
                    "Prioridade atribuída ao ROBOT por estar mais perto da entrada."
                )
                enviar_evento(
                    ID_CARRO_2,
                    "WAIT_FOR_ROBOT_PRIORITY",
                    {"REASON": "ROBOT_CLOSER_TO_EXIT"}
                )
                encaminhar_v2v(
                    ID_ROBOT,
                    ID_CARRO_2,
                    "ROBOT_HAS_EXIT_PRIORITY"
                )
            iniciar_saida_robot()
            return

        if carro_2_saiu_lugar:
            # CAR_2 has left space 2 (>= 15 cm, three readings),
            # but request the safe spot only after it reaches the CORNER.
            # Until then, track the manual exit in the approach state.
            # This applies to the ROBOT stopped at the entrance after START.
            if robot_parado_na_entrada_sem_rfid():
                print(
                    "CARRO 2 iniciou saída do Lugar 2. "
                    "Mova-o manualmente APENAS até ao CANTO e PARE nesse ponto. "
                    "O ROBOT só receberá P quando o canto for confirmado."
                )
                enviar_evento(
                    ID_CARRO_2,
                    "MANUAL_EXIT_TO_CORNER",
                    {"WAIT_AT_CORNER": "YES", "REASON": "ROBOT_AT_ENTRANCE"}
                )
            iniciar_saida_carro_2()
            return

    if estado_sistema == AGUARDAR_RFID_CARRO_2_DURANTE_SAIDA_ROBOT:
        if not saida_robot_pausa_confirmada and pode_mostrar_mensagem():
            print("A aguardar ACK PAUSE_EXIT do ROBOT; não mova CARRO 2.")
        elif pode_mostrar_mensagem():
            print("ROBOT suspenso. Apresente cartão RFID do CARRO 2 na entrada.")
        return

    if estado_sistema == AGUARDAR_ENTRADA_LIVRE_PARA_RETOMAR_SAIDA_ROBOT:
        if contador_entrada_livre < CONFIRMACOES_SENSOR:
            if pode_mostrar_mensagem():
                print("ROBOT parado. Aguardar CARRO 2 no SAFE PLACE e entrada livre.")
            return
        if not cliente_ligado(ID_ROBOT) or not saida_robot_pausa_confirmada:
            print("ROBOT desligado ou pausa não confirmada: saída NÃO retomada.")
            return
        fase = saida_robot_estado_interrompido
        if fase is None:
            print("ERRO: não foi guardado o estado de saída a retomar.")
            mudar_estado(ERRO_PERCURSO)
            return
        with estado_lock:
            estado_persistente["vehicles"]["car_2"]["location"] = "safe_place"
        guardar_estado()
        enviar_evento(ID_CARRO_2, "SAFE_PLACE_CONFIRMED",
                     {"WAIT_UNTIL": "ROBOT_EXITED"})
        print("Entrada livre; CARRO 2 no safe place. Retomar fase: {}".format(fase))
        # The ESP32 may restart between H and J; consider resumption
        # valid only after the firmware sends ACK RESUME_EXIT COMPLETED.
        saida_robot_retomada_confirmada = False
        if enviar_comando_robot("J"):
            bloquear_trafego("ROBOT_A_AGUARDAR_ACK_RETOMA")
            mudar_estado(AGUARDAR_ACK_RETOMA_SAIDA_ROBOT)
        return

    if estado_sistema == AGUARDAR_ACK_RETOMA_SAIDA_ROBOT:
        if saida_robot_retomada_confirmada:
            fase = saida_robot_estado_interrompido
            saida_robot_estado_interrompido = None
            saida_robot_pausa_confirmada = False
            saida_robot_retomada_confirmada = False
            contador_canto_saida = 0
            bloquear_trafego("ROBOT_A_SAIR_APOS_CONFLITO")
            mudar_estado(fase)  # restart the timer: time spent waiting in the safe spot does not count
            if fase == ROBOT_A_SAIR_ATE_ENTRADA:
                print("SAÍDA MANUAL: J não cria tração; continue manualmente até ao canto.")
        elif tempo_no_estado() > 4.0:
            print("ERRO: sem ACK RESUME_EXIT. A ordenar STOP por segurança.")
            enviar_comando_robot("S")
            bloquear_trafego("ERRO_RETOMA_SAIDA_SEM_ACK")
            mudar_estado(ERRO_PERCURSO)
        return

    if estado_sistema == AGUARDAR_ROBOT_NO_SAFE_PLACE_PARA_SAIDA_AUTO_CARRO_2:
        if contador_entrada_livre >= CONFIRMACOES_SENSOR:
            with estado_lock:
                estado_persistente["vehicles"]["robot"]["location"] = (
                    "safe_place"
                )
            guardar_estado()
            enviar_evento(
                ID_CARRO_2, "ROBOT_SAFE_PLACE_CONFIRMED",
                {"ENTRANCE": "CLEAR", "MODE": "AUTO_EXIT"}
            )
            encaminhar_v2v(
                ID_ROBOT, ID_CARRO_2, "ENTRANCE_CLEARED",
                {"MODE": "AUTO_EXIT"}
            )
            print(
                "Robot confirmado no safe place; entrada livre. "
                "CARRO 2 já está no CANTO e pode agora seguir "
                "manualmente até à ENTRADA para apresentar o RFID de saída."
            )
            with estado_lock:
                estado_persistente["vehicles"]["car_2"]["location"] = "corner"
            guardar_estado()
            enviar_evento(
                ID_CARRO_2,
                "MANUAL_EXIT_TO_ENTRANCE",
                {"FROM": "CORNER", "NEXT": "ENTRANCE", "MODE": "AUTO_EXIT"}
            )
            mudar_estado(CARRO_2_SAIDA_COMANDO_ATE_ENTRADA)
        return

    if estado_sistema == AGUARDAR_ENTRADA_LIVRE_PARA_START_APOS_CARRO_2:
        if contador_entrada_livre >= CONFIRMACOES_SENSOR:
            if not cliente_ligado(ID_ROBOT):
                if pode_mostrar_mensagem():
                    print(
                        "Entrada livre, mas o ESP32 do ROBOT está desligado. "
                        "O START continua pendente."
                    )
                return

            with estado_lock:
                robot_inside = bool(
                    estado_persistente["vehicles"]["robot"]["inside"]
                )
                robot_location = (
                    estado_persistente["vehicles"]["robot"]["location"]
                )

            if robot_inside or robot_location != "outside":
                print(
                    "START pendente cancelado por inconsistência do estado "
                    "do ROBOT: inside={} location='{}'.".format(
                        robot_inside,
                        robot_location
                    )
                )
                desbloquear_trafego()
                mudar_estado(AGUARDAR_VEICULO_ENTRADA)
                return

            print(
                "Entrada confirmada livre após o CARRO 2 avançar. "
                "A retomar automaticamente o comando START do ROBOT."
            )

            with estado_lock:
                estado_persistente["vehicles"]["robot"]["location"] = (
                    "approaching_entrance"
                )
                estado_persistente["vehicles"]["robot"]["slot"] = 0
            guardar_estado()

            bloquear_trafego(
                "ROBOT_START_E_CARRO_2_A_ESTACIONAR"
            )

            enviar_evento(
                ID_ROBOT,
                "START_RESUMED",
                {
                    "FROM": "OUTSIDE",
                    "DESTINATION": "ENTRANCE",
                    "REASON": "ENTRANCE_CLEAR"
                }
            )

            if enviar_comando_robot("F"):
                mudar_estado(ROBOT_START_ATE_ENTRADA)
            else:
                with estado_lock:
                    estado_persistente["vehicles"]["robot"]["location"] = (
                        "outside"
                    )
                guardar_estado()
                print(
                    "Não foi possível enviar F ao ROBOT. "
                    "O START continuará pendente e será tentado novamente "
                    "quando existir nova iteração com a entrada livre."
                )
            return
        return

    if estado_sistema == ROBOT_START_ATE_ENTRADA:
        # Unexpected conflict: CAR_2 started exiting while the ROBOT
        # is still approaching the entrance. Do not execute pivot P from an
        # unknown position: STOP and wait for intervention to avoid a collision.
        if carro_2_comecou_saida():
            enviar_comando_robot("S")
            print(
                "ALERTA DE SEGURANCA: CARRO 2 saiu do Lugar 2 durante "
                "a aproximação do ROBOT. Robot parado. Aguarde e reinicie "
                "o cenário em segurança."
            )
            bloquear_trafego("CONFLITO_CARRO_2_A_SAIR_COM_ROBOT_A_APROXIMAR")
            mudar_estado(ERRO_PERCURSO)
            return
        entrada_confirmada, segunda_leitura_entrada = (
            confirmar_entrada_start_rapido(entrada)
        )

        if entrada_confirmada:
            print(
                "START: ROBOT confirmado na entrada: primeira={} | "
                "segunda={} | limite={} cm. A enviar STOP.".format(
                    texto_distancia(entrada),
                    texto_distancia(segunda_leitura_entrada),
                    LIMITE_ENTRADA_CM
                )
            )

            if not enviar_comando_robot("S"):
                return

            with estado_lock:
                # Retain inside=False until RFID actually authorizes
                # entry into the parking area.
                estado_persistente["vehicles"]["robot"]["inside"] = False
                estado_persistente["vehicles"]["robot"]["location"] = "entrance"
            guardar_estado()

            registar_evento_historico(
                "ROBOT_ARRIVED_ENTRANCE_FROM_START",
                veiculo="robot",
                detalhes={
                    "first_distance_cm": entrada,
                    "second_distance_cm": segunda_leitura_entrada
                }
            )

            enviar_evento(
                ID_ROBOT,
                "START_ARRIVED_ENTRANCE",
                {"LOCATION": "ENTRANCE"}
            )

            print(
                "ROBOT parado na entrada. "
                "Apresente agora o chip RFID para continuar "
                "o estacionamento automático normal."
            )

            with estado_lock:
                carro_2_a_estacionar = (
                    estado_persistente["vehicles"]["car_2"]["location"]
                    == "parking"
                )
            if carro_2_a_estacionar and not corredor_robot_livre_para_rfid(distancias):
                # The ROBOT waits while CAR_2 occupies slots 1/2.
                bloquear_trafego(
                    "CARRO_2_A_ESTACIONAR_COM_ROBOT_A_AGUARDAR_RFID"
                )
                print(
                    "CARRO 2 ainda a estacionar: o monitor do Lugar 2 "
                    "permanece ativo em paralelo com AGUARDAR_RFID_ENTRADA."
                )
            else:
                # CAR_2 has already been confirmed in space 2 (or this is not the
                # relevant scenario). The Robot waits for its own RFID without reserving
                # the route, although the physical entrance remains occupied.
                desbloquear_trafego()
                print(
                    "START concluido na entrada: TRAFEGO LIVRE; "
                    "ROBOT aguarda RFID, sem iniciar a rota ao canto."
                )
            mudar_estado(AGUARDAR_RFID_ENTRADA)
            return

        if tempo_no_estado() >= TEMPO_MAXIMO_START_ATE_ENTRADA:
            print(
                "ERRO START: o ROBOT não chegou à entrada "
                "dentro do tempo máximo."
            )
            enviar_comando_robot("S")

            with estado_lock:
                estado_persistente["vehicles"]["robot"]["location"] = "outside"
            guardar_estado()

            mudar_estado(ERRO_PERCURSO)
        return

    if estado_sistema == AGUARDAR_VEICULO_ENTRADA:
        if contador_entrada >= CONFIRMACOES_SENSOR:
            if estado_persistente["traffic"]["blocked"]:
                return
            if not esta_livre(lugar_1, LIMITE_LUGAR_1_CM) and not lugar_2_livre_confirmavel(lugar_2):
                return
            print("Veículo detetado na entrada. Apresente o RFID.")
            mudar_estado(AGUARDAR_RFID_ENTRADA)
        return
    if estado_sistema == AGUARDAR_RFID_ENTRADA:
        if contador_entrada_livre >= CONFIRMACOES_SENSOR:
            # Important special case for the new START scenario:
            # the ROBOT has reached the entrance and stopped but has NOT presented RFID.
            # If physically removed from the entrance before identification,
            # it is no longer "entrance" or "inside"; it returns to "outside".
            with estado_lock:
                robot_inside = bool(
                    estado_persistente["vehicles"]["robot"]["inside"]
                )
                robot_location = (
                    estado_persistente["vehicles"]["robot"]["location"]
                )

            if (not robot_inside) and robot_location == "entrance":
                with estado_lock:
                    estado_persistente["vehicles"]["robot"]["inside"] = False
                    estado_persistente["vehicles"]["robot"]["location"] = (
                        "outside"
                    )
                guardar_estado()

                registar_evento_historico(
                    "ROBOT_LEFT_ENTRANCE_WITHOUT_RFID",
                    veiculo="robot",
                    detalhes={
                        "previous_location": "entrance",
                        "new_location": "outside",
                        "inside": False
                    }
                )

                enviar_evento(
                    ID_ROBOT,
                    "ENTRANCE_LEFT_WITHOUT_RFID",
                    {"LOCATION": "OUTSIDE"}
                )

                print(
                    "ROBOT saiu da entrada sem apresentar RFID. "
                    "Estado atualizado para location='outside'. "
                    "O comando 'start' pode ser executado novamente."
                )

            mudar_estado(AGUARDAR_VEICULO_ENTRADA)
        return
    if estado_sistema == CARRO_2_A_ESTACIONAR:
        if contador_lugar_2_ocupado >= CONFIRMACOES_SENSOR:
            with estado_lock:
                estado_persistente["vehicles"]["car_2"]["inside"] = True
                estado_persistente["vehicles"]["car_2"]["location"] = "place_2"
                estado_persistente["vehicles"]["car_2"]["slot"] = 3
                estado_persistente["parking"]["place_2"]["occupied"] = True
                estado_persistente["parking"]["place_2"]["vehicle"] = "car_2"
                estado_persistente["parking"]["place_2"]["reserved_by"] = None
            desbloquear_trafego()
            guardar_estado()
            registar_evento_historico(
                "PARKING_COMPLETED",
                veiculo="car_2",
                detalhes={"place": "place_2"}
            )
            enviar_evento(ID_CARRO_2, "PARKING_COMPLETED", {"PLACE": "PLACE_2"})
            encaminhar_v2v(ID_CARRO_2, ID_ROBOT, "CAR_2_PARKED", {"PLACE": "PLACE_2"})
            print("Carro 2 estacionado no Lugar 2. Sistema disponível para outra entrada.")
            mudar_estado(AGUARDAR_VEICULO_ENTRADA)
        return
    if estado_sistema == ROBOT_AVANCAR_ATE_CANTO:
        # First reading: use the value already obtained in the normal cycle.
        # If it is <= 16 cm, immediately take a second reading
        # ONLY from the corner sensor, 25 ms later.
        canto_confirmado, segunda_leitura_canto = (
            confirmar_canto_rapido(canto)
        )

        if canto_confirmado:
            print(
                "Canto confirmado rapidamente: primeira={} | segunda={} "
                "| limite={} cm".format(
                    texto_distancia(canto),
                    texto_distancia(segunda_leitura_canto),
                    LIMITE_CANTO_ENTRADA_CM
                )
            )

            if not enviar_comando_robot("S"):
                return
            time.sleep(PAUSA_ANTES_COMANDO_CURVA)
            if enviar_comando_robot("R"):
                with estado_lock:
                    estado_persistente["vehicles"]["robot"]["slot"] = 2
                guardar_estado()
                mudar_estado(ROBOT_AVANCAR_ATE_LUGAR_1)
            return
        if tempo_no_estado() >= TEMPO_MAXIMO_ATE_CANTO:
            enviar_comando_robot("S")
            mudar_estado(ERRO_PERCURSO)
        return
    if estado_sistema == ROBOT_AVANCAR_ATE_LUGAR_1:
        # First reading: use the value already obtained in the normal sensor cycle.
        # If it is <= 15 cm, immediately take a second reading
        # ONLY from the space 1 sensor, 25 ms later.
        lugar_1_confirmado, segunda_leitura_lugar_1 = (
            confirmar_lugar_1_rapido(lugar_1)
        )

        if lugar_1_confirmado:
            print(
                "Lugar 1 confirmado rapidamente: primeira={} | segunda={} "
                "| limite={} cm".format(
                    texto_distancia(lugar_1),
                    texto_distancia(segunda_leitura_lugar_1),
                    LIMITE_LUGAR_1_ESTACIONAMENTO_CM
                )
            )

            enviar_comando_robot("S")
            with estado_lock:
                estado_persistente["vehicles"]["robot"]["inside"] = True
                estado_persistente["vehicles"]["robot"]["location"] = "place_1"
                estado_persistente["vehicles"]["robot"]["slot"] = 2
                estado_persistente["parking"]["place_1"]["occupied"] = True
                estado_persistente["parking"]["place_1"]["vehicle"] = "robot"
            desbloquear_trafego()
            guardar_estado()
            registar_evento_historico(
                "PARKING_COMPLETED",
                veiculo="robot",
                detalhes={"place": "place_1"}
            )
            enviar_evento(ID_ROBOT, "PARKING_COMPLETED", {"PLACE": "PLACE_1"})
            encaminhar_v2v(ID_ROBOT, ID_CARRO_2, "ROBOT_PARKED", {"PLACE": "PLACE_1"})
            print("Robot estacionado no Lugar 1. Sistema disponível para outra entrada.")
            mudar_estado(AGUARDAR_VEICULO_ENTRADA)
            return
        if tempo_no_estado() >= TEMPO_MAXIMO_ATE_LUGAR_1:
            enviar_comando_robot("S")
            mudar_estado(ERRO_PERCURSO)
        return
    if estado_sistema == ESTACIONADO_LUGAR_1:
        if pode_mostrar_mensagem():
            print("Robot estacionado no Lugar 1 | distância={} | contador_livre={}/{}".format(texto_distancia(lugar_1), contador_lugar_1_livre, CONFIRMACOES_SENSOR))
        return
    if estado_sistema == ESTACIONADO_LUGAR_2:
        return
    if estado_sistema == AGUARDAR_ENTRADA_LIVRE_PARA_SAIR1:
        # The Robot has NOT yet left space 1. Only wait for
        # Car 2 to be moved manually from the entrance to the safe spot.
        if contador_entrada_livre >= CONFIRMACOES_SENSOR:
            if contador_lugar_1_ocupado < CONFIRMACOES_SENSOR:
                print(
                    "ERRO SAIR1: a entrada ficou livre, mas o sensor do "
                    "Lugar 1 já não confirma o ROBOT estacionado."
                )
                desbloquear_trafego()
                mudar_estado(ERRO_PERCURSO)
                return

            if not esta_livre(canto, LIMITE_CANTO_SAIDA_CM):
                print(
                    "SAIR1 continua em espera: o canto deixou de estar livre."
                )
                return

            print(
                "SAIR1: entrada agora livre. CARRO 2 considerado no safe place. "
                "A saída do ROBOT vai começar."
            )

            with estado_lock:
                estado_persistente["vehicles"]["car_2"]["location"] = (
                    "safe_place"
                )
            guardar_estado()

            enviar_evento(
                ID_CARRO_2,
                "SAFE_PLACE_CONFIRMED",
                {"ENTRANCE": "CLEAR", "PENDING_COMMAND": "SAIR1"}
            )
            encaminhar_v2v(
                ID_CARRO_2,
                ID_ROBOT,
                "ENTRANCE_CLEARED",
                {"PENDING_COMMAND": "SAIR1"}
            )

            return executar_sair1_com_entrada_livre()

        if pode_mostrar_mensagem():
            print(
                "SAIR1: o ROBOT permanece no Lugar 1. "
                "A aguardar que mova o CARRO 2 da entrada para o safe place."
            )
        return

    if estado_sistema == AGUARDAR_ENTRADA_LIVRE_PARA_SAIR2:
        # Car 2 has NOT yet left space 2. The Robot is executing
        # maneuver P; authorize the exit only after the entrance is clear.
        if contador_entrada_livre >= CONFIRMACOES_SENSOR:
            if contador_lugar_2_ocupado < CONFIRMACOES_SENSOR:
                print(
                    "ERRO SAIR2: a entrada ficou livre, mas o sensor do "
                    "Lugar 2 já não confirma o CARRO 2 estacionado."
                )
                desbloquear_trafego()
                mudar_estado(ERRO_PERCURSO)
                return

            if not esta_livre(canto, LIMITE_CANTO_SAIDA_CM):
                print(
                    "SAIR2 continua em espera: o canto deixou de estar livre."
                )
                return

            print(
                "SAIR2: entrada agora livre. ROBOT considerado no safe place. "
                "O movimento manual do CARRO 2 pode começar."
            )

            with estado_lock:
                estado_persistente["vehicles"]["robot"]["location"] = (
                    "safe_place"
                )
            guardar_estado()

            enviar_evento(
                ID_CARRO_2,
                "ROBOT_SAFE_PLACE_CONFIRMED",
                {"ENTRANCE": "CLEAR", "PENDING_COMMAND": "SAIR2"}
            )
            encaminhar_v2v(
                ID_ROBOT,
                ID_CARRO_2,
                "ENTRANCE_CLEARED",
                {"PENDING_COMMAND": "SAIR2"}
            )

            return executar_sair2_com_entrada_livre()

        if pode_mostrar_mensagem():
            print(
                "SAIR2: o CARRO 2 permanece no Lugar 2. "
                "A aguardar que o ROBOT liberte a entrada e chegue ao safe place."
            )
        return

    if estado_sistema == ROBOT_SAIDA_COMANDO_ATE_CANTO:
        if esta_ocupado(canto, LIMITE_CANTO_SAIR1_CM):
            contador_canto_saida += 1
        else:
            contador_canto_saida = 0

        if contador_canto_saida >= CONFIRMACOES_CANTO_SAIR1:
            print(
                "SAIR1: ROBOT detetado no canto a 20 cm ou menos, após 2 confirmações. "
                "A parar e a ordenar curva para a esquerda."
            )

            if not enviar_comando_robot("S"):
                return

            time.sleep(PAUSA_ANTES_COMANDO_CURVA)

            if enviar_comando_robot("L"):
                with estado_lock:
                    estado_persistente["vehicles"]["robot"]["location"] = (
                        "exiting_from_corner"
                    )
                guardar_estado()
                mudar_estado(ROBOT_SAIDA_COMANDO_ATE_ENTRADA)
            return

        if tempo_no_estado() >= TEMPO_MAXIMO_SAIDA_ATE_CANTO:
            print(
                "ERRO SAIR1: o ROBOT não chegou ao sensor do canto "
                "dentro do tempo máximo."
            )
            enviar_comando_robot("S")
            mudar_estado(ERRO_PERCURSO)
        return

    if estado_sistema == ROBOT_SAIDA_COMANDO_ATE_ENTRADA:
        # First reading: use the normal cycle's sample.
        # If it is <= 20 cm, immediately take a second reading
        # ONLY from the entrance sensor, 25 ms later.
        entrada_confirmada, segunda_leitura_entrada = (
            confirmar_entrada_sair1_rapido(entrada)
        )

        if entrada_confirmada:
            print(
                "SAIR1: entrada confirmada rapidamente: primeira={} | "
                "segunda={} | limite={} cm. "
                "A parar e a aguardar RFID.".format(
                    texto_distancia(entrada),
                    texto_distancia(segunda_leitura_entrada),
                    LIMITE_ENTRADA_SAIR1_CM
                )
            )

            if not enviar_comando_robot("S"):
                return

            with estado_lock:
                estado_persistente["vehicles"]["robot"]["location"] = (
                    "entrance"
                )
            guardar_estado()

            enviar_evento(
                ID_ROBOT,
                "RFID_EXIT_REQUIRED",
                {"LOCATION": "ENTRANCE", "MODE": "SAIR1"}
            )
            mudar_estado(AGUARDAR_RFID_SAIDA_ROBOT)
            return

        if tempo_no_estado() >= TEMPO_MAXIMO_SAIDA_ATE_ENTRADA:
            print(
                "ERRO SAIR1: o ROBOT não chegou ao sensor da entrada "
                "dentro do tempo máximo."
            )
            enviar_comando_robot("S")
            mudar_estado(ERRO_PERCURSO)
        return

    if estado_sistema == ROBOT_A_SAIR_ATE_ENTRADA:
        # The Robot has left space 1 and is approaching the corner.
        # For the ROBOT's exit, initiate the turn at 30 cm.
        if esta_ocupado(canto, LIMITE_CANTO_SAIDA_ROBOT_CM):
            contador_canto_saida += 1
        else:
            contador_canto_saida = 0

        if contador_canto_saida >= CONFIRMACOES_SENSOR:
            carro_2_na_entrada = esta_ocupado(
                entrada,
                LIMITE_ENTRADA_CM
            )

            if carro_2_na_entrada:
                print(
                    "Mova o Carro 2 até ao safe place. "
                    "O robot aguarda no canto."
                )
                bloquear_trafego(
                    "CARRO_2_DEVE_LIBERTAR_ENTRADA_PARA_SAIDA_ROBOT"
                )
                enviar_evento(
                    ID_CARRO_2,
                    "MOVE_TO_SAFE_PLACE",
                    {
                        "REASON": "ROBOT_EXITING",
                        "ROBOT_LOCATION": "CORNER"
                    }
                )
                enviar_evento(
                    ID_ROBOT,
                    "WAIT_AT_CORNER",
                    {"REASON": "CAR_2_AT_ENTRANCE"}
                )
                encaminhar_v2v(
                    ID_ROBOT,
                    ID_CARRO_2,
                    "CLEAR_ENTRANCE_REQUIRED"
                )
                mudar_estado(
                    AGUARDAR_CARRO_2_NO_SAFE_PLACE
                )
            else:
                enviar_evento(
                    ID_ROBOT,
                    "EXIT_CONTINUE",
                    {"NEXT": "RFID"}
                )
                mudar_estado(
                    ROBOT_A_SAIR_APOS_CANTO
                )
        return

    if estado_sistema == AGUARDAR_CARRO_2_NO_SAFE_PLACE:
        if contador_entrada_livre >= CONFIRMACOES_SENSOR:
            print(
                "Entrada livre. O robot pode avançar "
                "do canto até à entrada/saída."
            )
            with estado_lock:
                # Car 2 remains outside the parking area but is temporarily
                # in the safe spot.
                if not estado_persistente["vehicles"]["car_2"]["inside"]:
                    estado_persistente["vehicles"]["car_2"]["location"] = "safe_place"
            guardar_estado()
            bloquear_trafego("ROBOT_A_SAIR")
            enviar_evento(
                ID_CARRO_2,
                "SAFE_PLACE_CONFIRMED",
                {"ENTRANCE": "CLEAR"}
            )
            enviar_evento(
                ID_ROBOT,
                "EXIT_CONTINUE",
                {"NEXT": "RFID"}
            )
            encaminhar_v2v(
                ID_CARRO_2,
                ID_ROBOT,
                "ENTRANCE_CLEARED"
            )
            mudar_estado(
                ROBOT_A_SAIR_APOS_CANTO
            )
        elif pode_mostrar_mensagem():
            print(
                "A aguardar que o Carro 2 liberte a entrada "
                "e seja movido para o safe place."
            )
        return

    if estado_sistema == ROBOT_A_SAIR_APOS_CANTO:
        if contador_entrada_saida_robot >= CONFIRMACOES_SENSOR:
            print(
                "ROBOT detetado na entrada durante a saída "
                "a 20 cm ou menos."
            )
            enviar_evento(
                ID_ROBOT,
                "RFID_EXIT_REQUIRED",
                {"LOCATION": "ENTRANCE"}
            )
            mudar_estado(
                AGUARDAR_RFID_SAIDA_ROBOT
            )
        return

    if estado_sistema == AGUARDAR_RFID_SAIDA_ROBOT:
        return
    if estado_sistema == CARRO_2_SAIDA_COMANDO_ATE_CANTO:
        if esta_ocupado(canto, LIMITE_CANTO_SAIDA_CM):
            contador_canto_saida += 1
        else:
            contador_canto_saida = 0

        # While the vehicle moves, update the JSON when space 2 becomes free.
        if contador_lugar_2_livre >= CONFIRMACOES_SENSOR:
            with estado_lock:
                estado_persistente["parking"]["place_2"]["occupied"] = False
                estado_persistente["parking"]["place_2"]["vehicle"] = None
            guardar_estado()

        if contador_canto_saida >= CONFIRMACOES_SENSOR:
            print(
                "SAIR2: CARRO 2 detetado no CANTO. "
                "Agora mova-o manualmente até à ENTRADA/SAÍDA."
            )

            with estado_lock:
                estado_persistente["vehicles"]["car_2"]["location"] = "corner"
            guardar_estado()

            enviar_evento(
                ID_CARRO_2,
                "MANUAL_EXIT_TO_ENTRANCE",
                {
                    "FROM": "CORNER",
                    "NEXT": "ENTRANCE"
                }
            )
            mudar_estado(CARRO_2_SAIDA_COMANDO_ATE_ENTRADA)
        elif pode_mostrar_mensagem():
            print(
                "SAIR2: mova manualmente o CARRO 2 do Lugar 2 até ao canto."
            )
        return

    if estado_sistema == CARRO_2_SAIDA_COMANDO_ATE_ENTRADA:
        if contador_entrada >= CONFIRMACOES_SENSOR:
            print(
                "SAIR2: CARRO 2 detetado na ENTRADA a {}. "
                "Apresente agora o cartão RFID do CARRO 2.".format(
                    texto_distancia(entrada)
                )
            )

            with estado_lock:
                estado_persistente["vehicles"]["car_2"]["location"] = "entrance"
            guardar_estado()

            enviar_evento(
                ID_CARRO_2,
                "RFID_EXIT_REQUIRED",
                {
                    "LOCATION": "ENTRANCE",
                    "COMMAND": "SAIR2"
                }
            )
            mudar_estado(AGUARDAR_RFID_SAIDA_CARRO_2)
        elif pode_mostrar_mensagem():
            print(
                "SAIR2: aguarda-se o CARRO 2 na entrada/saída."
            )
        return

    if estado_sistema == CARRO_2_A_SAIR_ATE_CANTO:
        # When leaving space 2, Car 2 must be stopped manually when
        # detected at the corner. Only then request use of the safe spot.
        if esta_ocupado(canto, LIMITE_CANTO_SAIDA_CM):
            contador_canto_saida += 1
        else:
            contador_canto_saida = 0
        if contador_canto_saida >= CONFIRMACOES_SENSOR:
            robot_na_entrada = esta_ocupado(entrada, LIMITE_ENTRADA_CM)
            if robot_na_entrada and robot_parado_na_entrada_sem_rfid():
                preparar_saida_auto_carro_2_com_robot_na_entrada(distancias)
                return
            if robot_na_entrada:
                print(
                    "SAIDA CARRO 2 BLOQUEADA: sensor de entrada ocupado "
                    "mas veículo não identificado como Robot em espera. "
                    "Mantenha o CARRO 2 parado antes da entrada."
                )
                return
            enviar_evento(ID_CARRO_2, "EXIT_CONTINUE", {"NEXT": "RFID"})
            mudar_estado(AGUARDAR_RFID_SAIDA_CARRO_2)
        return
    if estado_sistema == AGUARDAR_RFID_SAIDA_CARRO_2:
        return
    if estado_sistema == AGUARDAR_ENTRADA_LIVRE_PARA_RETORNO_ROBOT:
        # Car 2 has already presented its exit RFID but may still be
        # physically in front of the sensor. Send B only after it leaves that position.
        if contador_entrada_livre >= CONFIRMACOES_SENSOR:
            print(
                "Entrada livre após a saída do CARRO 2. "
                "A ordenar ao ROBOT o regresso inverso do SAFE PLACE."
            )

            if regressar_robot_do_safe_place():
                bloquear_trafego("ROBOT_A_RETORNAR_ENTRADA")
                mudar_estado(AGUARDAR_ROBOT_RETORNAR_ENTRADA)
            elif pode_mostrar_mensagem():
                print(
                    "Não foi possível iniciar o regresso do ROBOT. "
                    "Nova tentativa será feita."
                )
        return

    if estado_sistema == AGUARDAR_ENTRADA_LIVRE_PARA_RETORNO_CARRO_2:
        # In addition to a clear entrance, wait for ACTUAL completion of the
        # final 2-second forward movement (firmware ACK EXIT_TO_START).
        if not robot_avanco_final_confirmado:
            if pode_mostrar_mensagem():
                print("CARRO 2 continua no safe place; aguarda-se ACK do avanço final de 2s.")
            return
        if contador_entrada_livre >= CONFIRMACOES_SENSOR:
            print(
                "Entrada livre após a saída do ROBOT. "
                "Mova agora manualmente o CARRO 2 do SAFE PLACE "
                "para a ENTRADA."
            )

            with estado_lock:
                estado_persistente["vehicles"]["car_2"]["location"] = (
                    "returning_to_entrance"
                )
            guardar_estado()

            bloquear_trafego("CARRO_2_A_RETORNAR_ENTRADA")

            enviar_evento(
                ID_CARRO_2,
                "RETURN_TO_ENTRANCE",
                {
                    "FROM": "SAFE_PLACE",
                    "DESTINATION": "ENTRANCE"
                }
            )

            mudar_estado(AGUARDAR_CARRO_2_RETORNAR_ENTRADA)
        return

    if estado_sistema == AGUARDAR_CARRO_2_RETORNAR_ENTRADA:
        if contador_entrada >= CONFIRMACOES_SENSOR:
            print(
                "CARRO 2 detetado novamente na entrada. "
                "Pode apresentar o cartão RFID para efetuar a entrada."
            )

            with estado_lock:
                estado_persistente["vehicles"]["car_2"]["location"] = "entrance"
            guardar_estado()

            desbloquear_trafego()

            enviar_evento(
                ID_CARRO_2,
                "RETURN_TO_ENTRANCE_CONFIRMED",
                {"LOCATION": "ENTRANCE"}
            )

            mudar_estado(AGUARDAR_RFID_ENTRADA)
        elif pode_mostrar_mensagem():
            print(
                "A aguardar que o CARRO 2 seja movido manualmente "
                "do safe place para a entrada."
            )
        return

    if estado_sistema == AGUARDAR_ROBOT_RETORNAR_ENTRADA:
        # After Car 2 exits, the Robot continues reversing.
        # The first reading comes from the normal cycle. If it is <= 15 cm,
        # immediately take a second reading ONLY from the entrance,
        # 25 ms later.
        entrada_retorno_confirmada, segunda_leitura_retorno = (
            confirmar_entrada_retorno_rapido(entrada)
        )

        if entrada_retorno_confirmada:
            print(
                "Retorno: entrada confirmada rapidamente: primeira={} | "
                "segunda={} | limite={} cm. "
                "A enviar comando de paragem.".format(
                    texto_distancia(entrada),
                    texto_distancia(segunda_leitura_retorno),
                    LIMITE_ENTRADA_CM
                )
            )

            if enviar_comando_robot("S"):
                with estado_lock:
                    estado_persistente["vehicles"]["robot"]["location"] = (
                        "entrance"
                    )
                guardar_estado()

                enviar_evento(
                    ID_ROBOT,
                    "RETURN_COMPLETED",
                    {
                        "LOCATION": "ENTRANCE",
                        "STOP_REASON": "ENTRY_SENSOR_15_CM"
                    }
                )
                transmitir_evento_infraestrutura(
                    "ROBOT_RETURNED_TO_ENTRANCE"
                )

                desbloquear_trafego()

                print(
                    "ROBOT regressou à entrada. "
                    "Apresente o chip RFID para efetuar a entrada."
                )
                mudar_estado(AGUARDAR_RFID_ENTRADA)
            else:
                print(
                    "Falha ao enviar S ao robot. "
                    "Nova tentativa será feita na próxima leitura."
                )
        return

    if estado_sistema == AGUARDAR_ENTRADA_LIVRE_APOS_SAIDA:
        if contador_entrada_livre >= CONFIRMACOES_SENSOR:
            transmitir_evento_infraestrutura("ENTRANCE_AVAILABLE")
            mudar_estado(AGUARDAR_VEICULO_ENTRADA)


def loop_principal_parque():
    """
    Sensor loop and state machine.
    Run in a dedicated thread to leave the main thread available
    for the keyboard interface.
    """
    global programa_ativo

    while programa_ativo:
        try:
            verificar_timeouts_clientes()

            # Critical state: during the Corner -> Entrance segment of SAIR1,
            # prioritize only the entrance sensor to avoid the normal cycle's
            # approximately 300 ms interval between readings.
            if estado_sistema == ROBOT_SAIDA_COMANDO_ATE_ENTRADA:
                processar_polling_rapido_entrada_sair1()
                continue

            distancias = medir_todos_sensores()

            mostrar_distancias(distancias)
            processar_maquina_estados(distancias)

            # Execute sair1/sair2 here using these current readings.
            processar_comandos_interface(distancias)

            uid, texto = obter_leitura_rfid()
            if uid is not None:
                processar_rfid(uid, texto, distancias)

            time.sleep(PAUSA_LOOP)

        except Exception as erro:
            print("Erro no loop principal do parque: {}".format(erro))
            programa_ativo = False
            return


def main():
    global programa_ativo

    thread_parque = None

    try:
        carregar_estado()
        configurar_gpio()
        iniciar_servidor_tcp()
        iniciar_thread_rfid()

        print("\n" + "=" * 72)
        print("SISTEMA DE ESTACIONAMENTO — MULTI-CLIENTE V2X")
        print("=" * 72)
        print("HELLO esperado do robot: HELLO|ID=ROBOT|TYPE=AUTONOMOUS")
        print("HELLO esperado do carro2: HELLO|ID=CAR_2|TYPE=CONNECTED_MANUAL")
        print("Estado persistente: parking_state.json")
        print("Histórico não crítico: parking_events.jsonl")

        thread_parque = threading.Thread(
            target=loop_principal_parque,
            daemon=True
        )
        thread_parque.start()

        # Keep input() in the main thread so it works correctly.
        interface_comandos_bloqueante()

    except KeyboardInterrupt:
        print("\nPrograma terminado pelo utilizador.")

    except Exception as erro:
        print("Erro geral: {}".format(erro))

    finally:
        programa_ativo = False

        if thread_parque is not None:
            thread_parque.join(timeout=2.0)

        if cliente_ligado(ID_ROBOT):
            try:
                enviar_comando_robot("S")
            except Exception:
                pass

        fechar_todos_clientes()

        if servidor_tcp is not None:
            try:
                servidor_tcp.close()
            except OSError:
                pass

        GPIO.cleanup()
        print("GPIO e ligações encerrados.")


if __name__ == "__main__":
    main()
