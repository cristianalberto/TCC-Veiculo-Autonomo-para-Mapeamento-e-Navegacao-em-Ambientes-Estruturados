# ===== ESP32 MicroPython — Lógica do carrinho (MQTT + UART) =====
# - MQTT (umqtt.simple) para enviar matriz e receber comandos.
# - UART para conversar com o Arduino (movimento + sensores).
# - Mapeamento em matriz com:
#     0 = desconhecido
#     1 = livre visitado
#     2 = bloqueado / inacessível
#     3 = posição atual do robô
# - Busca do próximo '0' leva em conta custo de movimento:
#     custo = 1 por giro (90°) + 1 por passo à frente.
# - Detecção de confinamento: flood fill a partir da posição atual;
#   se não encostar na borda => considera ambiente confinado e
#   marca todos os '0' fora da região alcançável como '2'.
# - Expansão da matriz apenas quando o robô encostar na borda,
#   crescendo +1 célula em cada lado, com limite máximo 30x30.
# - Antes de girar para uma nova direção (planejamento normal), o ESP32
#   solicita ao Arduino medição do lado correspondente (servo + ultrassom)
#   e só gira se a distância medida for > 14 cm.
# - BLOQUEIO FRONTAL:
#     se o Arduino retornar "caminho bloqueado":
#         → marca célula à frente como '2'
#         → auto_step chama tratar_bloqueio_frontal(), que:
#             - mede direita e esquerda;
#             - marca laterais bloqueadas na matriz;
#             - se ambos lados bloqueados → dá ré;
#             - se algum lado livre → vira para esse lado.
#
# UART1: TX=17, RX=16 a 115200 baud.

from machine import Pin, UART, unique_id
from time import sleep, sleep_ms
import time, gc, _thread
import network, ubinascii, ujson
from umqtt.simple import MQTTClient

# -------- Config Wi-Fi/MQTT --------
WIFI_SSID     = "TP_LINK"
WIFI_PASSWORD = "20agosto80"
BROKER_IP     = "3.140.173.157"
BROKER_PORT   = 1883

TOPIC_MATRIX_FULL = b"comando/matriz"
TOPIC_MATRIX_PART = b"comando/matriz/part"
TOPIC_CMD_ESP     = b"comando/esp"

MAX_PART_SIZE     = 900
CLIENT_ID         = b'esp32-' + ubinascii.hexlify(unique_id())

# -------- UART p/ Arduino --------
uart = UART(1, 115200)
uart.init(115200, bits=8, parity=None, stop=1, tx=17, rx=16)

# -------- Estado do mapa --------
TAM_INICIAL_MATRIZ = 25
TAM_MAX_MATRIZ     = 30  # limite 30x30

localizacao = []
pos_x = pos_y = 0
direcao_atual = 'up'
confinamento_detectado = False

def resetar_mapa():
    global localizacao, pos_x, pos_y, direcao_atual, confinamento_detectado
    tam = TAM_INICIAL_MATRIZ
    localizacao = [[0 for _ in range(tam)] for _ in range(tam)]
    pos_x = tam // 2
    pos_y = tam // 2
    localizacao[pos_x][pos_y] = 3
    direcao_atual = 'up'
    confinamento_detectado = False
    gc.collect()

resetar_mapa()

# -------- Estados de operação --------
Modelo1 = 'AUTO'
Modelo2 = 'MANUAL'
Modelo   = Modelo2

movimento_em_andamento = False
esperando_confirmacao  = False
movimento_bloqueado    = False

# -------- MQTT client --------
_mqtt = None

# ======== Wi-Fi ========
def connect_wifi(timeout_s=20):
    wlan = network.WLAN(network.STA_IF)
    if not wlan.active():
        wlan.active(True)
    if not wlan.isconnected():
        print("Conectando Wi-Fi:", WIFI_SSID)
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        t0 = time.ticks_ms()
        while (not wlan.isconnected() and
               time.ticks_diff(time.ticks_ms(), t0) < timeout_s*1000):
            print(".", end="")
            sleep_ms(300)
        print("")
    if wlan.isconnected():
        print("Wi-Fi OK:", wlan.ifconfig())
        return True
    print("Wi-Fi falhou")
    return False

# ======== Helpers de direção ========
def delta_da_dir(d):
    return {'up':(-1,0),'right':(0,1),'down':(1,0),'left':(0,-1)}[d]

def dir_esquerda(d):
    mapa = {'up':'left','left':'down','down':'right','right':'up'}
    return mapa[d]

def dir_direita(d):
    mapa = {'up':'right','right':'down','down':'left','left':'up'}
    return mapa[d]

# ======== MQTT ========
def _on_msg(topic, msg):
    global Modelo
    try:
        t = topic.decode() if isinstance(topic, (bytes, bytearray)) else str(topic)
        if t == TOPIC_CMD_ESP.decode():
            d = ujson.loads(msg)
            if d.get("cmd") == "auto":
                value = bool(d.get("value"))
                novo_modelo = Modelo1 if value else Modelo2
                if Modelo != novo_modelo and value:
                    print("Religando modo automático: resetando mapa.")
                    resetar_mapa()
                Modelo = novo_modelo
                uart.write(('MODO_AUTOMATICO\n' if value else 'MODO_MANUAL\n'))
                print("CMD AUTO =>", "LIGADO" if value else "DESLIGADO")
    except Exception as e:
        print("Erro callback MQTT:", e)

def mqtt_connect():
    global _mqtt
    c = MQTTClient(client_id=CLIENT_ID, server=BROKER_IP,
                   port=BROKER_PORT, keepalive=60)
    c.set_callback(_on_msg)
    c.connect()
    c.subscribe(TOPIC_CMD_ESP)
    _mqtt = c
    print("MQTT conectado; inscrito em", TOPIC_CMD_ESP.decode())

def mqtt_check():
    global _mqtt
    try:
        if _mqtt:
            _mqtt.check_msg()
    except Exception as e:
        print("MQTT erro:", e)
        try:
            _mqtt.disconnect()
        except:
            pass
        sleep(1)
        while not connect_wifi(timeout_s=10):
            print("Re-tentando Wi-Fi após erro MQTT em 5s...")
            sleep(5)
        while True:
            try:
                mqtt_connect()
                break
            except Exception as e2:
                print("reconnect MQTT falhou:", e2)
                sleep(5)

def _serialize_matrix():
    try:
        return ujson.dumps({
            "matrix": localizacao,
            "confinado": confinamento_detectado
        })
    except Exception as e:
        print("Falha JSON:", e)
        return None

def publish_matrix():
    if not _mqtt:
        return
    payload = _serialize_matrix()
    if not payload:
        return
    if len(payload) <= MAX_PART_SIZE:
        _mqtt.publish(TOPIC_MATRIX_FULL, payload)
        return True

    parts, cur = [], []
    for row in localizacao:
        test = cur + [row]
        cand = ujson.dumps({
            "id": CLIENT_ID.decode(), "seq": 0,
            "total": 0, "data": test, "confinado": confinamento_detectado
        })
        if len(cand) < MAX_PART_SIZE:
            cur = test
        else:
            parts.append(cur)
            cur = [row]
    if cur:
        parts.append(cur)
    total = len(parts)
    for i, block in enumerate(parts, 1):
        payload = ujson.dumps({
            "id": CLIENT_ID.decode(),
            "seq": i,
            "total": total,
            "data": block,
            "confinado": confinamento_detectado
        })
        _mqtt.publish(TOPIC_MATRIX_PART, payload)
        sleep_ms(30)
    return True

def thread_publicador():
    while True:
        try:
            mqtt_check()
            publish_matrix()
        except Exception as e:
            print("Erro publicador:", e)
        sleep(0.3)

# ======== Matriz: expansão e confinamento ========
def verificar_necessidade_expansao():
    global pos_x, pos_y, localizacao, confinamento_detectado
    tam = len(localizacao)
    if confinamento_detectado:
        return False
    if tam >= TAM_MAX_MATRIZ:
        return False
    if pos_x == 0 or pos_x == tam-1 or pos_y == 0 or pos_y == tam-1:
        print("Na borda; será necessário expandir (se possível).")
        return True
    return False

def expandir_matriz():
    global localizacao, pos_x, pos_y
    t = len(localizacao)
    if t >= TAM_MAX_MATRIZ:
        print("Tamanho máximo de matriz atingido; não expande.")
        return False
    novo = min(TAM_MAX_MATRIZ, t + 2)
    print("Expandindo matriz", t, "→", novo)
    nova = [[0 for _ in range(novo)] for _ in range(novo)]
    off = (novo - t) // 2
    for i in range(t):
        for j in range(t):
            nova[i + off][j + off] = localizacao[i][j]
    pos_x += off
    pos_y += off
    localizacao[:] = nova
    gc.collect()
    return True

def verificar_e_expandir_matriz():
    if verificar_necessidade_expansao():
        return expandir_matriz()
    return False

def verificar_confinamento():
    global confinamento_detectado, localizacao, pos_x, pos_y

    if confinamento_detectado:
        return True

    tam = len(localizacao)
    visitados = set()
    fila = [(pos_x, pos_y)]
    visitados.add((pos_x, pos_y))
    toca_borda = False
    i = 0

    while i < len(fila):
        x, y = fila[i]
        i += 1
        if x == 0 or y == 0 or x == tam-1 or y == tam-1:
            toca_borda = True
            break
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < tam and 0 <= ny < tam:
                if (nx, ny) not in visitados and localizacao[nx][ny] != 2:
                    visitados.add((nx, ny))
                    fila.append((nx, ny))

    if toca_borda:
        return False

    alterou = False
    for i in range(tam):
        for j in range(tam):
            if localizacao[i][j] == 0 and (i, j) not in visitados:
                localizacao[i][j] = 2
                alterou = True

    confinamento_detectado = True
    if alterou:
        print("Confinamento detectado: marcando exterior como inacessível (2).")
        publish_matrix()
    else:
        print("Confinamento detectado (sem zeros externos para marcar).")
    return True

# ======== UART / Movimento ========
def enviar_comando_arduino(cmd):
    global movimento_em_andamento, esperando_confirmacao
    print("CMD ->", cmd)
    uart.write(cmd + '\n')
    if cmd in ('MOVER_FRENTE', 'VIRAR_ESQUERDA', 'VIRAR_DIREITA', 'MOVER_TRAS'):
        movimento_em_andamento = True
        esperando_confirmacao = True
        timeout = 0
        while esperando_confirmacao and timeout < 100:
            processar_resposta_arduino()
            sleep_ms(100)
            timeout += 1
        movimento_em_andamento = False

def processar_resposta_arduino():
    global pos_x, pos_y, direcao_atual
    global esperando_confirmacao, localizacao, movimento_bloqueado

    if not uart.any():
        return

    try:
        resp = uart.read().decode('utf-8').strip()
    except:
        resp = ""

    if not resp:
        return

    gc.collect()
    print("RESP <-", resp)

    if "carrinho: frente" in resp:
        localizacao[pos_x][pos_y] = 1
        dx, dy = delta_da_dir(direcao_atual)
        pos_x += dx
        pos_y += dy
        verificar_e_expandir_matriz()
        localizacao[pos_x][pos_y] = 3
        esperando_confirmacao = False
        movimento_bloqueado = False
        publish_matrix()

    elif "carrinho: virando esquerda" in resp:
        direcao_atual = dir_esquerda(direcao_atual)
        esperando_confirmacao = False
        movimento_bloqueado = False
        publish_matrix()

    elif "carrinho: virando direita" in resp:
        direcao_atual = dir_direita(direcao_atual)
        esperando_confirmacao = False
        movimento_bloqueado = False
        publish_matrix()

    elif "carrinho: voltou" in resp:
        localizacao[pos_x][pos_y] = 1
        dx, dy = delta_da_dir(direcao_atual)
        pos_x -= dx
        pos_y -= dy
        verificar_e_expandir_matriz()
        localizacao[pos_x][pos_y] = 3
        esperando_confirmacao = False
        movimento_bloqueado = False
        publish_matrix()

    elif "caminho bloqueado" in resp:
        dx, dy = delta_da_dir(direcao_atual)
        ox, oy = pos_x + dx, pos_y + dy
        if 0 <= ox < len(localizacao) and 0 <= oy < len(localizacao[0]):
            localizacao[ox][oy] = 2
        else:
            verificar_e_expandir_matriz()
        esperando_confirmacao = False
        movimento_bloqueado = True
        publish_matrix()

    gc.collect()

def girar_para(desejada):
    global direcao_atual
    ordem = ['up','right','down','left']
    idx_atual = ordem.index(direcao_atual)
    idx_dest  = ordem.index(desejada)
    diff = (idx_dest - idx_atual) % 4
    if diff == 0:
        return True
    elif diff == 1:
        enviar_comando_arduino('VIRAR_DIREITA')
    elif diff == 2:
        enviar_comando_arduino('VIRAR_DIREITA')
        enviar_comando_arduino('VIRAR_DIREITA')
    elif diff == 3:
        enviar_comando_arduino('VIRAR_ESQUERDA')
    return True

def mover_frente():
    enviar_comando_arduino('MOVER_FRENTE')

def executar_voltar():
    print("Executar recuo de segurança (MOVER_TRAS).")
    enviar_comando_arduino('MOVER_TRAS')

# ======== Medição de distância nos lados (Arduino) ========
def medir_lado(lado):
    if lado == 'direita':
        cmd = 'MEDIR_DIREITA'
        chave = 'DIST_DIREITA'
    else:
        cmd = 'MEDIR_ESQUERDA'
        chave = 'DIST_ESQUERDA'

    print("CMD ->", cmd)
    uart.write(cmd + '\n')

    timeout = 0
    while timeout < 80:
        sleep_ms(50)
        if not uart.any():
            timeout += 1
            continue
        try:
            resp = uart.read().decode('utf-8')
        except:
            resp = ""
        if not resp:
            timeout += 1
            continue

        print("RESP <-", resp.strip())
        if chave in resp:
            try:
                parte = resp.split(chave + ":")[1]
                num_str = ""
                for ch in parte:
                    if ch.isdigit():
                        num_str += ch
                    else:
                        break
                if num_str:
                    dist = int(num_str)
                    print("Distância lado", lado, "=", dist, "cm")
                    return dist
            except:
                pass
            return 999
    print("Timeout ao medir lado", lado)
    return 999

def verificar_caminho_lateral_seguro(prox_dir, limite_cm=14):
    global pos_x, pos_y, direcao_atual, localizacao

    ordem = ['up','right','down','left']
    idx_atual = ordem.index(direcao_atual)
    idx_dest  = ordem.index(prox_dir)
    diff = (idx_dest - idx_atual) % 4

    if diff == 0 or diff == 2:
        return True

    lado = 'direita' if diff == 1 else 'esquerda'
    dist = medir_lado(lado)
    print("Distância lado", lado, "=", dist, "cm")

    dx, dy = delta_da_dir(prox_dir)
    nx, ny = pos_x + dx, pos_y + dy

    if dist <= 0:
        print("Leitura inválida; assume bloqueio preventivo.")
        if 0 <= nx < len(localizacao) and 0 <= ny < len(localizacao[0]):
            localizacao[nx][ny] = 2
            publish_matrix()
        return False

    if dist < limite_cm:
        print("Caminho lateral", lado, "bloqueado (<{} cm)".format(limite_cm))
        if 0 <= nx < len(localizacao) and 0 <= ny < len(localizacao[0]):
            localizacao[nx][ny] = 2
            publish_matrix()
        return False

    print("Lado", lado, "livre, permitido girar.")
    return True

# ======== Marcar laterais bloqueadas na matriz ========
def marcar_lado_bloqueado_na_matriz(lado, dist, limite_cm=14):
    """
    Marca na matriz a célula lateral (direita ou esquerda) como 2
    se a distância medida for menor que limite_cm.
    """
    global pos_x, pos_y, direcao_atual, localizacao

    if dist <= 0 or dist >= limite_cm:
        return  # só marcamos se de fato estiver perto/bloqueado

    if lado == 'direita':
        dir_lateral = dir_direita(direcao_atual)
    else:
        dir_lateral = dir_esquerda(direcao_atual)

    dx, dy = delta_da_dir(dir_lateral)
    nx, ny = pos_x + dx, pos_y + dy

    if 0 <= nx < len(localizacao) and 0 <= ny < len(localizacao[0]):
        if localizacao[nx][ny] != 2:
            print("Marcando lado", lado, "como bloqueado na matriz em ({}, {})".format(nx, ny))
        localizacao[nx][ny] = 2
        publish_matrix()

# ======== Tratamento de bloqueio frontal (com análise de lados) ========
def tratar_bloqueio_frontal():
    """
    Novo comportamento:
      - Ao detectar 'caminho bloqueado' à frente, o robô:
          1) mede direita e esquerda;
          2) marca na matriz as laterais que estiverem < limite_cm como 2;
          3) se os dois lados estiverem bloqueados → recua (MOVER_TRAS);
          4) se pelo menos um lado for livre → gira para um lado livre
             (preferindo direita se ambos forem livres).
    """
    global movimento_bloqueado

    LIM = 14
    print("Bloqueio frontal detectado: analisando lados (direita -> esquerda).")

    # 1) mede direita
    dist_dir = medir_lado('direita')
    print("Distância lado direita (bloqueio frontal) =", dist_dir, "cm")
    marcar_lado_bloqueado_na_matriz('direita', dist_dir, limite_cm=LIM)

    # 2) mede esquerda
    dist_esq = medir_lado('esquerda')
    print("Distância lado esquerda (bloqueio frontal) =", dist_esq, "cm")
    marcar_lado_bloqueado_na_matriz('esquerda', dist_esq, limite_cm=LIM)

    dir_livre = (dist_dir >= LIM)
    esq_livre = (dist_esq >= LIM)

    # 3) decisão: ambos bloqueados?
    if (not dir_livre) and (not esq_livre):
        print("Ambos lados bloqueados (<{} cm); recuando.".format(LIM))
        executar_voltar()
    else:
        # Pelo menos um lado livre — escolhe lado
        if dir_livre and not esq_livre:
            print("Direita livre (>= {} cm), virando para direita.".format(LIM))
            enviar_comando_arduino('VIRAR_DIREITA')
        elif esq_livre and not dir_livre:
            print("Esquerda livre (>= {} cm), virando para esquerda.".format(LIM))
            enviar_comando_arduino('VIRAR_ESQUERDA')
        else:
            print("Ambos lados livres (>= {} cm); preferindo virar para direita.")
            enviar_comando_arduino('VIRAR_DIREITA')

    movimento_bloqueado = False

# ======== Planejamento simples ========
def bfs_rota(orig, dest):
    (sx, sy) = orig
    (tx, ty) = dest
    tam = len(localizacao)
    dq = [(sx, sy)]
    prev = {(sx, sy): None}
    i = 0
    while i < len(dq):
        x, y = dq[i]
        i += 1
        if (x, y) == (tx, ty):
            break
        for d in ('up', 'right', 'down', 'left'):
            dx, dy = delta_da_dir(d)
            nx, ny = x + dx, y + dy
            if 0 <= nx < tam and 0 <= ny < tam and (nx, ny) not in prev:
                if localizacao[nx][ny] != 2:
                    prev[(nx, ny)] = (x, y, d)
                    dq.append((nx, ny))
    if (tx, ty) not in prev:
        return None
    path_dirs = []
    cur = (tx, ty)
    while prev[cur] is not None:
        px, py, d = prev[cur]
        path_dirs.append(d)
        cur = (px, py)
    path_dirs.reverse()
    return path_dirs

def proximo_zero():
    tam = len(localizacao)
    ini = (pos_x, pos_y, direcao_atual)
    fila = [ini]
    visit = {ini}
    i = 0
    while i < len(fila):
        x, y, d = fila[i]
        i += 1
        if localizacao[x][y] == 0:
            return (x, y)
        dx, dy = delta_da_dir(d)
        nx, ny = x + dx, y + dy
        if (0 <= nx < tam and 0 <= ny < tam and
            localizacao[nx][ny] != 2):
            st = (nx, ny, d)
            if st not in visit:
                visit.add(st)
                fila.append(st)
        d_esq = dir_esquerda(d)
        st = (x, y, d_esq)
        if st not in visit:
            visit.add(st)
            fila.append(st)
        d_dir = dir_direita(d)
        st = (x, y, d_dir)
        if st not in visit:
            visit.add(st)
            fila.append(st)
    return None

def existe_zero_na_matriz():
    for linha in localizacao:
        if 0 in linha:
            return True
    return False

# ======== Modo automático ========
def auto_step():
    global Modelo, movimento_bloqueado

    # 1) Se houve bloqueio frontal recente, primeiro trata bloqueio
    if movimento_bloqueado:
        tratar_bloqueio_frontal()
        gc.collect()
        return True

    # 2) Não há mais zeros: mapeamento concluído
    if not existe_zero_na_matriz():
        print("Mapeamento concluído: não há mais '0' na matriz.")
        Modelo = Modelo2
        publish_matrix()
        return False

    # 3) Atualiza estado de confinamento
    verificar_confinamento()

    # 4) Encontra zero mais próximo em termos de custo (giro + passo)
    alvo = proximo_zero()
    if not alvo:
        print("Nenhum '0' alcançável a partir da posição atual.")
        if existe_zero_na_matriz():
            print("Beco aparente: executando recuo para tentar nova rota.")
            executar_voltar()
            return True
        else:
            print("Sem zeros e sem rotas; encerrando.")
            Modelo = Modelo2
            publish_matrix()
            return False

    rota = bfs_rota((pos_x, pos_y), alvo)
    if not rota or len(rota) == 0:
        ax, ay = alvo
        localizacao[ax][ay] = 2
        print("Sem rota para alvo", alvo, "- marcando como inacessível (2).")
        publish_matrix()
        return True

    prox_dir = rota[0]

    # 5) Executa o primeiro passo do plano
    if prox_dir != direcao_atual:
        if not verificar_caminho_lateral_seguro(prox_dir):
            print("Caminho lateral bloqueado; replanejamento ficará para próxima iteração.")
            return True
        girar_para(prox_dir)
    else:
        mover_frente()

    sleep_ms(200)
    return True

# ======== Loop principal ========
def main_loop():
    print("Sistema iniciado — Modo automático DESLIGADO; aguarda comando em 'comando/esp'")
    while True:
        mqtt_check()
        processar_resposta_arduino()
        if Modelo == Modelo1:
            try:
                auto_step()
            except Exception as e:
                print("Erro em auto_step:", e)
            gc.collect()
        sleep_ms(50)

# ======== Boot ========
while not connect_wifi(timeout_s=15):
    print("Falha ao conectar Wi-Fi; tentando novamente em 5s...")
    sleep(5)

while True:
    try:
        mqtt_connect()
        break
    except Exception as e:
        print("Falha MQTT inicial:", e)
        print("Tentando MQTT novamente em 5s...")
        sleep(5)

_thread.start_new_thread(thread_publicador, ())
main_loop()

