# TCC-Veiculo-Autonomo-para-Mapeamento-e-Navegacao-em-Ambientes-Estruturados

# 🤖 Veículo Autônomo para Mapeamento e Navegação

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)
[![MicroPython](https://img.shields.io/badge/MicroPython-ESP32-green.svg)](https://micropython.org/)
[![Arduino](https://img.shields.io/badge/Arduino-Uno-teal.svg)](https://www.arduino.cc/)

Sistema de robótica móvel autônoma para mapeamento e navegação em ambientes estruturados, desenvolvido como Trabalho de Conclusão de Curso em Engenharia Elétrica.

---

## 📋 Resumo

Este trabalho apresenta uma **arquitetura distribuída** para mapeamento e navegação autônoma de veículos robóticos, integrando dois microcontroladores com responsabilidades distintas. O **ESP32** atua como unidade de processamento de alto nível, executando algoritmos de mapeamento incremental, planejamento de trajetórias e comunicação via Wi-Fi. O **ATmega328P** (Arduino Uno) gerencia o controle de baixo nível dos atuadores e sensores em tempo real.

O sistema emprega **rodas omnidirecionais** para movimentação holonômica, permitindo deslocamentos laterais e rotações no próprio eixo. A percepção do ambiente é realizada por um **sensor ultrassônico** montado em **servo motor**, possibilitando varredura angular do cone frontal de visão. O mapeamento é construído de forma incremental utilizando uma representação em **grade de ocupação bidimensional**, com aplicação do algoritmo **flood fill** para detecção de regiões confinadas e fronteiras exploráveis.

Para o planejamento de trajetórias, foi implementado um algoritmo de **busca em largura (BFS) em espaço tridimensional**, onde os estados incluem não apenas a posição (x, y), mas também a **orientação do robô**. Esta abordagem permite encontrar caminhos ótimos considerando o custo diferenciado de rotações versus movimentos lineares, adequando-se às limitações dinâmicas do veículo.

A comunicação com a infraestrutura em nuvem utiliza o protocolo **MQTT** hospedado em instância **AWS EC2**, com implementação de **fragmentação automática de mensagens** para contornar limitações do broker. Uma **API REST** desenvolvida em **FastAPI** e uma **interface web** com **WebSockets** permitem monitoramento em tempo real da matriz de mapeamento e envio de comandos ao robô.

Todos os algoritmos desenvolvidos em **MicroPython** foram **validados computacionalmente no Google Colab** antes da implementação no hardware, demonstrando correção lógica no mapeamento incremental, detecção de confinamento e planejamento de rotas. O trabalho documenta de forma didática a integração completa entre hardware e software para robótica móvel, servindo como referência para reprodução e extensão do sistema.

As limitações identificadas incluem derrapagem das rodas omnidirecionais em superfícies lisas e sensibilidade do sensor ultrassônico a características do ambiente. Como trabalhos futuros, propõe-se a adição de **odometria por encoders**, **sensores inerciais (IMU)** e implementação de algoritmos SLAM para mapeamento mais robusto.

---

## 🎯 Características Principais

### 🧠 Algoritmos Implementados

#### 1️⃣ Mapeamento Incremental com Flood Fill
- **Grade de ocupação 2D** com células de estados: livre, ocupado, desconhecido
- **Atualização incremental** conforme o robô se move e coleta leituras
- **Algoritmo Flood Fill** para identificar regiões confinadas e fronteiras exploráveis
- Detecção automática de **becos sem saída** e otimização de exploração

#### 2️⃣ Busca em Largura (BFS) em Espaço 3D
- Planejamento de trajetórias considerando **posição (x, y) + orientação (θ)**
- Estados tridimensionais: `(x, y, orientação)`
- Transições: avançar, girar 90° esquerda, girar 90° direita
- **Caminho ótimo** com menor número de ações
- Adequado para movimentação holonômica

#### 3️⃣ Comunicação MQTT com Fragmentação
- Protocolo **MQTT** para comunicação robô ↔ nuvem
- **Fragmentação automática** de mensagens grandes (matriz de mapeamento)
- Reconstituição no servidor via múltiplos tópicos
- QoS configurável e tratamento de desconexões

#### 4️⃣ Controle Reativo de Obstáculos
- Varredura ultrassônica em **três ângulos** (frente, 30° direita, 30° esquerda)
- Detecção preventiva de colisões
- Recuo automático e replanejamento de rota
- **Múltiplas amostras** com filtragem estatística para robustez

---

## 🏗️ Arquitetura do Sistema

### Distribuição de Responsabilidades
```
┌─────────────────────────────────────────────────────────────┐
│                         ESP32                               │
│  • Mapeamento (Flood Fill)                                  │
│  • Planejamento (BFS 3D)                                    │
│  • Comunicação Wi-Fi/MQTT                                   │
│  • Lógica de alto nível                                     │
└──────────────────┬──────────────────────────────────────────┘
                   │ UART (115200 bps)
                   │ Protocolo texto ASCII
┌──────────────────┴──────────────────────────────────────────┐
│                    Arduino Uno (ATmega328P)                 │
│  • Controle de motores (74HC595 + L293D)                    │
│  • Leitura sensor ultrassônico                              │
│  • Controle servo motor                                     │
│  • Loop de controle em tempo real                           │
└─────────────────────────────────────────────────────────────┘
```

### Comunicação em Nuvem
```
┌──────────┐   MQTT    ┌──────────┐  REST/WS  ┌──────────────┐
│  ESP32   │ ←────────→│  Broker  │←─────────→│  Interface   │
│  Robô    │           │  AWS EC2 │           │     Web      │
└──────────┘           └──────────┘           └──────────────┘
                            │
                            │ FastAPI
                            ↓
                       ┌──────────┐
                       │  Backend │
                       │  Python  │
                       └──────────┘
```

---

## 🛠️ Hardware

### Componentes Principais

- **ESP32-CAM**: Processamento principal, Wi-Fi, algoritmos
- **Arduino Uno** (ATmega328P): Controle de atuadores/sensores
- **4x Motores DC tipo TT** com caixa de redução
- **4x Rodas omnidirecionais** 60mm
- **Sensor ultrassônico HC-SR04**: Medição de distância (2cm-400cm)
- **Servo motor 9g**: Varredura angular do sensor
- **74HC595**: Registrador de deslocamento (expansão de I/O)
- **2x L293D**: Pontes H para controle bidirecional dos motores
- **2x Baterias 18650** (alimentação dual: motores + ESP32)

### Sistema de Alimentação Dual
```
┌─────────────┐         ┌──────────────────┐
│  Baterias   │────────→│  Regulador 5V    │──→ Arduino + Motores
│  18650      │         │  LM1086          │
└─────────────┘         └──────────────────┘

┌─────────────┐
│  Power Bank │────────────────────────────→ ESP32 (isolado)
└─────────────┘
```

**Justificativa**: Transientes de corrente dos motores causavam interferência no ESP32 quando compartilhavam o mesmo trilho de alimentação.

---

## 💻 Software

### Linguagens e Frameworks

- **MicroPython** (ESP32): Algoritmos de mapeamento e navegação
- **C/C++** (ATmega328P/Arduino Uno): Controle de hardware em tempo real
- **Python 3.8+** (Servidor): Backend FastAPI
- **JavaScript** (Frontend): Interface web com WebSockets
- **HTML/CSS**: Interface de visualização

---

## 🚀 Como Usar

### 1️⃣ Montagem do Hardware

1. Montar estrutura mecânica (chassis acrílico)
2. Instalar motores e rodas omnidirecionais
3. Conectar ESP32 e Arduino conforme diagrama
4. Configurar sistema de alimentação dual
5. Testar motores e sensores individualmente

### 2️⃣ Configuração do Servidor AWS
```bash
# Conectar à instância EC2
ssh -i sua-chave.pem ubuntu@IP_PUBLICO

# Instalar dependências
sudo apt update
sudo apt install mosquitto mosquitto-clients python3-pip

# Instalar broker MQTT
sudo systemctl start mosquitto
sudo systemctl enable mosquitto

# Configurar backend
cd servidor/
pip3 install -r requirements.txt
python3 main.py
```

### 3️⃣ Programação do Arduino
```bash
# Abrir Arduino IDE
# Carregar arquivos.
# Selecionar porta e placa (Arduino Uno)
# Upload
```

### 4️⃣ Programação do ESP32
```bash
# Instalar Thonny IDE
# Conectar ESP32 via USB
# Configurar interpretador: MicroPython (ESP32)

# Editar main.py com suas credenciais:
WIFI_SSID = "sua_rede"
WIFI_PASSWORD = "sua_senha"
MQTT_BROKER = "ip_do_servidor"

# Upload dos arquivos main.py para ESP32
```

### 5️⃣ Interface Web
```bash
# Abrir web/index.html no navegador
```

---

## 📊 Validação Computacional

Os algoritmos foram validados **antes da implementação no hardware** usando Google Colab:

---

## 🧪 Resultados

### ✅ Sucessos

- ✅ Mapeamento incremental funcional
- ✅ Detecção de regiões confinadas (flood fill)
- ✅ Planejamento de rotas ótimas (BFS 3D)
- ✅ Comunicação MQTT estável com fragmentação
- ✅ Interface web em tempo real
- ✅ Controle de motores omnidirecionais

### ⚠️ Limitações Identificadas

- 🔴 **Derrapagem em superfícies lisas**: Rodas omnidirecionais perdem aderência
- 🔴 **Odometria inexistente**: Deriva acumulativa sem encoders
- 🟡 **Sensor ultrassônico**: Sensível a materiais absorventes e ângulos
- 🟡 **Altura do sensor**: Não detecta obstáculos baixos

---

## 🔮 Trabalhos Futuros

### Hardware

1. **Encoders nas rodas**: Odometria para correção de deriva
2. **IMU (MPU6050)**: Giroscópio + acelerômetro para orientação precisa
3. **Sensor LIDAR**: Varredura 360° de alta resolução
4. **Rodas com maior aderência**: Reduzir derrapagem

### Software

1. **SLAM (Simultaneous Localization and Mapping)**: Mapeamento robusto com correção de trajetória
2. **Filtro de Kalman**: Fusão sensorial (ultrassônico + encoders + IMU)
3. **A\* em espaço 3D**: Planejamento mais eficiente que BFS
4. **Replanejamento dinâmico**: Atualização de rota durante navegação

---

## 📖 Documentação Adicional

- 📘 [Manual de Montagem Completo](docs/manual_montagem.md)
- 📗 [Protocolo UART ESP32-Arduino](docs/protocolo_uart.md)
- 📙 [Configuração AWS EC2](docs/aws_setup.md)
- 📕 [API REST Endpoints](docs/api_reference.md)

---

## 👨‍🎓 Autor

**Cristian Alberto Gimenez de Castro**  
Trabalho de Conclusão de Curso  
Engenharia Elétrica - Universidade Estadual de Londrina  
Ano: 2025

📧 Email: cristian.alberto12@gmail.com  
🐙 GitHub: @cristianalberto (https://github.com/cristianalberto)

---

## 🙏 Agradecimentos

- Prof. Dr. Leonimer Flávio de Melo - Orientação e suporte técnico
- Universidade Estadual de Londrina (UEL) - Infraestrutura e recursos
- Comunidade MicroPython e Arduino 
- AWS Education - Créditos para hospedagem

---

## 📚 Referências

1. LaValle, S. M. (2006). *Planning Algorithms*. Cambridge University Press.
2. Cormen, T. H. et al. (2009). *Introduction to Algorithms* (3rd ed.). MIT Press.
3. Siegwart, R., & Nourbakhsh, I. R. (2004). *Introduction to Autonomous Mobile Robots*. MIT Press.

---

**⭐ Se este projeto foi útil, considere dar uma estrela!**
