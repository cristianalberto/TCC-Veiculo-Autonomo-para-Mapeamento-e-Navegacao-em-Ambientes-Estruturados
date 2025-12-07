#include <Servo.h>

// Servo control pin
#define MOTOR_PIN           9
// PWM control pin
#define PWM1_PIN            5
#define PWM2_PIN            6
// 74HCT595N chip pin
#define SHCP_PIN            2
#define EN_PIN              7
#define DATA_PIN            8
#define STCP_PIN            4
// Ultrasonic control pins
#define Trig_PIN            12
#define Echo_PIN            13

Servo MOTORservo;

// Movement constants (padrão do kit)
const int Forward       = 92;
const int Backward      = 163;
const int Turn_Left     = 149;  // movimento lateral para a esquerda
const int Turn_Right    = 106;  // movimento lateral para a direita
const int Stop          = 0;
const int Contrarotate  = 172;  // rotação anti-horária (virar esquerda)
const int Clockwise     = 83;   // rotação horária (virar direita)

// Control variables
uint16_t angle = 90;
String comando_recebido = "";
int UT_distance = 0;

// ======================================================
//                      Setup
// ======================================================
void setup() {
    Serial.setTimeout(10);
    Serial.begin(115200);

    MOTORservo.attach(MOTOR_PIN);

    pinMode(SHCP_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);
    pinMode(DATA_PIN, OUTPUT);
    pinMode(STCP_PIN, OUTPUT);

    pinMode(PWM1_PIN, OUTPUT);
    pinMode(PWM2_PIN, OUTPUT);

    pinMode(Trig_PIN, OUTPUT);
    pinMode(Echo_PIN, INPUT);

    // Servo começa olhando para frente
    MOTORservo.write(angle);
    Motor(Stop, 0);
    
    Serial.println("Arduino iniciado - Aguardando comandos");
}

void loop() {
    receberComandos();
    delay(10);
}

// ======================================================
//                 Recebimento de comandos
// ======================================================
void receberComandos() {
    if (Serial.available() > 0) {
        comando_recebido = Serial.readStringUntil('\n');
        comando_recebido.trim();
        
        if (comando_recebido == "MODO_AUTOMATICO") {
            Serial.println("Modo automático indicado pelo ESP32");
            
        } else if (comando_recebido == "MODO_MANUAL") {
            Motor(Stop, 0);
            Serial.println("Modo manual indicado pelo ESP32");
            
        } else if (comando_recebido == "MOVER_FRENTE") {
            executarMovimentoFrente();
            
        } else if (comando_recebido == "MOVER_TRAS") {
            executarMovimentoTras();
            
        } else if (comando_recebido == "VIRAR_ESQUERDA") {
            executarVirarEsquerda();
            
        } else if (comando_recebido == "VIRAR_DIREITA") {
            executarVirarDireita();
            
        } else if (comando_recebido == "MEDIR_DISTANCIA") {
            medirEEnviarDistancia();
            
        } else if (comando_recebido == "MEDIR_DIREITA") {
            int d = medirDireita();
            Serial.print("DIST_DIREITA:");
            Serial.println(d);
            
        } else if (comando_recebido == "MEDIR_ESQUERDA") {
            int d = medirEsquerda();
            Serial.print("DIST_ESQUERDA:");
            Serial.println(d);
            
        } else if (comando_recebido == "ANALISAR_LADOS") {
            analisarLadosESensor();
            
        } else if (comando_recebido == "PARAR") {
            Motor(Stop, 0);
            Serial.println("Parado");
        }
        
        comando_recebido = "";
    }
}

// ======================================================
//      Funções de segurança para rotação lateral
// ======================================================
//
// Ideia:
//   - Para virar ESQUERDA: olhar fixo para a DIREITA (0°) e
//     deslocar lateralmente para a ESQUERDA (Turn_Left) até
//     a distância à direita ser >= 15 cm.
//   - Para virar DIREITA: olhar fixo para a ESQUERDA (180°) e
//     deslocar lateralmente para a DIREITA (Turn_Right) até
//     a distância à esquerda ser >= 15 cm.
//
// E usamos medição por MEDIANA, com algumas leituras descartadas.

// Mede o HC-SR04 uma vez (tempo de voo -> cm)
float SR04(int Trig, int Echo) {
    digitalWrite(Trig, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trig, LOW);

    unsigned long duration = pulseIn(Echo, HIGH, 30000);  // Timeout 30ms
    
    if (duration == 0) {
        return 400;  // Timeout - valor alto
    }
    
    float distance = duration / 58.00;
    if (distance > 400) {
        distance = 400;
    }
    return distance;
}

// Mede distância com o servo apontando para um ângulo específico,
// descartando algumas amostras iniciais e usando a MEDIANA das leituras.
int medirPosicionadoMediana(int anguloServo, int amostras, int descartes) {
    if (amostras < 1) amostras = 1;
    if (amostras > 15) amostras = 15;  // limite de segurança
    
    // Posiciona o servo
    MOTORservo.write(anguloServo);
    delay(300);  // tempo para o servo estabilizar

    // Descarta leituras iniciais (aquecimento / ecos ruins da posição anterior)
    for (int i = 0; i < descartes; i++) {
        SR04(Trig_PIN, Echo_PIN);
        delay(10);
    }

    // Coleta leituras válidas
    int valores[15];
    int count = 0;
    int tentativas = 0;
    const int TENTATIVAS_MAX = amostras * 3; // para evitar loop infinito

    while (count < amostras && tentativas < TENTATIVAS_MAX) {
        int d = (int)SR04(Trig_PIN, Echo_PIN);
        if (d > 0 && d < 400) {
            valores[count++] = d;
        }
        tentativas++;
        delay(10);
    }

    if (count == 0) {
        // Nenhuma leitura confiável
        return 400;
    }

    // Ordena os valores coletados (bubble sort simples para vetores pequenos)
    for (int i = 0; i < count - 1; i++) {
        for (int j = 0; j < count - 1 - i; j++) {
            if (valores[j] > valores[j + 1]) {
                int tmp = valores[j];
                valores[j] = valores[j + 1];
                valores[j + 1] = tmp;
            }
        }
    }

    // Retorna a mediana
    int mediana = valores[count / 2];
    return mediana;
}

// Medição usando o ângulo ATUAL do servo (não altera o ângulo)
int medirServoAtualMediana(int amostras, int descartes) {
    if (amostras < 1) amostras = 1;
    if (amostras > 15) amostras = 15;

    // Descarta leituras iniciais na posição atual
    for (int i = 0; i < descartes; i++) {
        SR04(Trig_PIN, Echo_PIN);
        delay(10);
    }

    int valores[15];
    int count = 0;
    int tentativas = 0;
    const int TENTATIVAS_MAX = amostras * 3;

    while (count < amostras && tentativas < TENTATIVAS_MAX) {
        int d = (int)SR04(Trig_PIN, Echo_PIN);
        if (d > 0 && d < 400) {
            valores[count++] = d;
        }
        tentativas++;
        delay(10);
    }

    if (count == 0) {
        return 400;
    }

    // Ordena para pegar mediana
    for (int i = 0; i < count - 1; i++) {
        for (int j = 0; j < count - 1 - i; j++) {
            if (valores[j] > valores[j + 1]) {
                int tmp = valores[j];
                valores[j] = valores[j + 1];
                valores[j + 1] = tmp;
            }
        }
    }
    int mediana = valores[count / 2];
    return mediana;
}

// Garante espaço mínimo de 13 cm no lado oposto antes de virar à direita
// (olha fixo para a ESQUERDA e anda lateral para a DIREITA).
void garantirEspacoParaVirarDireita() {
    const int LIMITE_CM = 13;
    const int MAX_TENTATIVAS = 5;

    Motor(Stop, 0);
    // Servo olha para a ESQUERDA (lado oposto ao giro)
    MOTORservo.write(180);
    delay(300);

    // Pequeno descarte inicial
    for (int i = 0; i < 3; i++) {
        SR04(Trig_PIN, Echo_PIN);
        delay(10);
    }

    int tent = 0;
    while (tent < MAX_TENTATIVAS) {
        int distEsq = medirServoAtualMediana(5, 0);  // mede no ângulo atual (180°)
        Serial.print("GARANTIA_VIRAR_DIR_DIST_ESQ:");
        Serial.println(distEsq);

        if (distEsq >= LIMITE_CM) {
            break;  // já tem espaço suficiente
        }

        // Muito perto da parede na esquerda: andar lateralmente para a direita
        Motor(Turn_Right, 250);
        delay(400);  // ajuste em campo
        Motor(Stop, 0);

        tent++;
    }

    // Volta servo para frente
    MOTORservo.write(90);
    delay(200);
}

// Garante espaço mínimo de 13 cm no lado oposto antes de virar à esquerda
// (olha fixo para a DIREITA e anda lateral para a ESQUERDA).
void garantirEspacoParaVirarEsquerda() {
    const int LIMITE_CM = 13;
    const int MAX_TENTATIVAS = 5;

    Motor(Stop, 0);
    // Servo olha para a DIREITA (lado oposto ao giro)
    MOTORservo.write(0);
    delay(300);

    // Pequeno descarte inicial
    for (int i = 0; i < 3; i++) {
        SR04(Trig_PIN, Echo_PIN);
        delay(10);
    }

    int tent = 0;
    while (tent < MAX_TENTATIVAS) {
        int distDir = medirServoAtualMediana(5, 0);  // mede no ângulo atual (0°)
        Serial.print("GARANTIA_VIRAR_ESQ_DIST_DIR:");
        Serial.println(distDir);

        if (distDir >= LIMITE_CM) {
            break;  // já tem espaço suficiente
        }

        // Muito perto da parede na direita: andar lateralmente para a esquerda
        Motor(Turn_Left, 250);
        delay(400);  // ajuste em campo
        Motor(Stop, 0);

        tent++;
    }

    // Volta servo para frente
    MOTORservo.write(90);
    delay(200);
}

// ======================================================
//          Movimentos básicos de uma “casa” e giros
// ======================================================
//
// Aqui entra o refinamento que você pediu:
//   - Antes de andar pra frente, faz três medições:
//       * 90° (frente) -> limite 25 cm
//       * 70° (20° à direita) -> limite 27 cm
//       * 110° (20° à esquerda) -> limite 27 cm
//     Se QUALQUER uma estiver abaixo do limite -> "caminho bloqueado"
//   - Freio emergencial durante o movimento: 10 cm

void executarMovimentoFrente() {
    // Cone frontal de segurança
    int dist_centro = medirPosicionadoMediana(90, 9, 3);   // frente
    int dist_dir20  = medirPosicionadoMediana(60, 9, 3);   // 20° à direita
    int dist_esq20  = medirPosicionadoMediana(120, 9, 3);  // 20° à esquerda

    Serial.print("DIST_FRENTE_90:");
    Serial.println(dist_centro);
    Serial.print("DIST_FRENTE_70:");
    Serial.println(dist_dir20);
    Serial.print("DIST_FRENTE_110:");
    Serial.println(dist_esq20);

    // Recentraliza o servo antes de andar
    MOTORservo.write(90);
    delay(200);

    // Limiares:
    // - 25 cm na frente
    // - 27 cm nas diagonais leves (70° e 110°)
    if (dist_centro < 25 || dist_dir20 < 27 || dist_esq20 < 27) {
        Serial.println("caminho bloqueado");
        return;
    }
    
    // Move para frente (1 célula) com freio de emergência em 10 cm
    Motor(Forward, 250);
    unsigned long t0 = millis();
    const unsigned long DURACAO_MS = 800;
    
    while (millis() - t0 < DURACAO_MS) {
        int d = (int)SR04(Trig_PIN, Echo_PIN);  // leitura rápida
        if (d > 0 && d < 10) {
            // Obstáculo muito perto: freio de emergência
            Motor(Stop, 0);
            Serial.println("caminho bloqueado");
            return;
        }
        delay(30);
    }
    
    Motor(Stop, 0);
    Serial.println("carrinho: frente");
}

void executarMovimentoTras() {
    // Ré não usa sensor (como combinado)
    Motor(Backward, 250);
    delay(800);
    Motor(Stop, 0);
    
    Serial.println("carrinho: voltou");
}

void executarVirarEsquerda() {
    // Antes de girar, garante espaço no lado oposto (direita)
    garantirEspacoParaVirarEsquerda();

    // Agora faz a rotação ~90° para a esquerda
    Motor(Contrarotate, 250);
    delay(700);  // ajuste esse valor no campo para ~90°
    Motor(Stop, 0);
    
    Serial.println("carrinho: virando esquerda");
}

void executarVirarDireita() {
    // Antes de girar, garante espaço no lado oposto (esquerda)
    garantirEspacoParaVirarDireita();

    // Agora faz a rotação ~90° para a direita
    Motor(Clockwise, 250);
    delay(700);  // ajuste esse valor no campo para ~90°
    Motor(Stop, 0);
    
    Serial.println("carrinho: virando direita");
}

// ======================================================
//            Funções de medição “públicas”
// ======================================================

void medirEEnviarDistancia() {
    UT_distance = medirPosicionadoMediana(90, 9, 3);
    Serial.print("DIST_FRENTE_MANUAL:");
    Serial.println(UT_distance);
}

// Medição lateral direita (servo vai a 0° e volta para 90°)
int medirDireita() {
    Motor(Stop, 0);
    delay(150);

    int dist = medirPosicionadoMediana(0, 9, 3);

    // Volta para frente
    MOTORservo.write(90);
    delay(200);

    return dist;
}

// Medição lateral esquerda (servo vai a 180° e volta para 90°)
int medirEsquerda() {
    Motor(Stop, 0);
    delay(150);

    int dist = medirPosicionadoMediana(180, 9, 3);

    // Volta para frente
    MOTORservo.write(90);
    delay(200);

    return dist;
}

// Mantida para debug, agora usa as funções robustas
void analisarLadosESensor() {
    bool esquerdaLivre = false;
    bool direitaLivre = false;

    int distEsquerda = medirEsquerda();
    if (distEsquerda > 35) {
        esquerdaLivre = true;
    }

    int distDireita = medirDireita();
    if (distDireita > 35) {
        direitaLivre = true;
    }

    String resultado = "ANALISE:";
    resultado += esquerdaLivre ? "true" : "false";
    resultado += ":";
    resultado += direitaLivre ? "true" : "false";

    Serial.println(resultado);

    Serial.print("Esquerda (");
    Serial.print(distEsquerda);
    Serial.print("cm): ");
    Serial.print(esquerdaLivre ? "LIVRE" : "BLOQUEADO");
    Serial.print(" | Direita (");
    Serial.print(distDireita);
    Serial.print("cm): ");
    Serial.println(direitaLivre ? "LIVRE" : "BLOQUEADO");
}

// ======================================================
//          Controle dos motores (74HC595 + PWM)
// ======================================================
void Motor(int Dir, int Speed) {
    digitalWrite(EN_PIN, LOW);
    analogWrite(PWM1_PIN, Speed);
    analogWrite(PWM2_PIN, Speed);

    digitalWrite(STCP_PIN, LOW);
    shiftOut(DATA_PIN, SHCP_PIN, MSBFIRST, Dir);
    digitalWrite(STCP_PIN, HIGH);
}
