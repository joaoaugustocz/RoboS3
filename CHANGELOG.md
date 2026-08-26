# Changelog

## 0.4.0

- interface unificada de distância por ID para dois HC-SR04 e até dois VL53L0X;
- pinagem padrão revisada dos ultrassons: ULTRA1 em 17/18 e ULTRA2 em 16/42;
- integração da biblioteca Pololu VL53L0X 1.3.1 com seleção entre I2C0 e I2C1,
  modo contínuo e indicação de timeout;
- detecção e diagnóstico de conflitos no endereço I2C `0x29` entre TCS34725 e
  VL53L0X, com possibilidade de desativar sensores de cor opcionais;
- leitura comum com centímetros, milímetros, tecnologia, validade, timeout,
  identificador e timestamp;
- exemplo 30 com cadastro de sensores e leitura independente da tecnologia.

## 0.3.0

- seleção configurável de I2C para cada TCS34725 e para a MPU6050;
- diagnóstico explícito quando os dois TCS34725 são atribuídos ao mesmo I2C;
- giro bloqueante por ângulo com timeout, sentido positivo/negativo e inversão
  configurável do eixo Z;
- exemplo didático de estratégia OBR com calibração de cores, obstáculo,
  padrões de linha, PID, verde, encruzilhada e curvas de 90 graus;
- revisão documentada da pinagem QTR e ponte H e alerta de colisões ainda
  existentes entre os GPIOs provisórios de QTR, I2C0 e ultrassom;
- documentação Doxygen padronizada em toda a biblioteca, incluindo módulos,
  configurações, tipos, constantes, parâmetros, retornos e unidades;
- README reorganizado com instalação pela Arduino IDE, sumário, convenções da
  referência, unidades dos tipos retornados e orientação de manutenção.

## 0.2.0

- diagnóstico automático de módulos com falha durante `begin()`;
- 256 padrões de linha nomeados com `P/B`, como `BBBPPBBB`;
- histórico circular configurável das últimas mudanças de linha e suas durações;
- documentação Doxygen na API pública para exibição no hover da IDE;
- exemplos de histórico de linha e calibração manual de cor;
- documentação ampliada sobre retornos, memória e desempenho.

## 0.1.0

- primeira versão pública da biblioteca RoboS3.
