# TCC-Veiculo-Autonomo-para-Mapeamento-e-Navegacao-em-Ambientes-Estruturados

Este trabalho apresenta uma arquitetura distribuída para mapeamento e navegação autô-
noma integrando ESP32 e ATmega328P/Arduino Uno com comunicação UART. O sis-
tema utiliza rodas omnidirecionais, sensor ultrassônico em servo motor e implementa
algoritmos de busca em largura em espaço tridimensional para planejamento de traje-
tórias. A comunicação com servidor AWS EC2 emprega protocolo MQTT com frag-
mentação automática de mensagens, complementada por interface web com WebSockets
para monitoramento em tempo real. Os algoritmos desenvolvidos em MicroPython foram
validados computacionalmente no Google Colab, demonstrando correção no mapeamento
incremental e detecção de confinamento. O trabalho documenta a integração de hardware
e software para robótica móvel, apresentando soluções para comunicação entre diferentes
plataformas e identificando requisitos sensoriais adicionais para maior robustez, incluindo
odometria por encoders e instrumentação inercial.
