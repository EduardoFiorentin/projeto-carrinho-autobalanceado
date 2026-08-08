Sim. Li o projeto inteiro. A estrutura de software está **bem melhor do que o estágio atual do controle físico**: as responsabilidades estão separadas de forma limpa, mas existem alguns problemas conceituais que, do jeito atual, tornam muito improvável o carrinho realmente se autobalancear.

Minha avaliação geral seria:

| Parte                          | Estado                                                     |
| ------------------------------ | ---------------------------------------------------------- |
| Organização do código          | **Boa**                                                    |
| Driver do L298N                | **Funcional, mas precisa ajustes**                         |
| Leitura do MPU6050             | **Parcial**                                                |
| Filtro                         | **Funciona como suavizador, não como estimador de ângulo** |
| PID                            | **Implementação correta, variável controlada errada**      |
| PowerRamp                      | **Perigoso para balanceamento**                            |
| Temporização                   | **Aceitável para testes, melhorável**                      |
| Segurança/failsafe             | **Praticamente inexistente**                               |
| Algoritmo de autobalanceamento | **Ainda incompleto**                                       |

## 1. Arquitetura do projeto: boa

Aqui eu manteria praticamente tudo.

Seu `sketch.ino` ficou exatamente como eu gosto para esse tipo de projeto:

```cpp
SelfBalancingRobot robot;

void setup() {
    robot.begin();
}

void loop() {
    robot.update();
}
```

Ou seja, o Arduino só inicia e atualiza a aplicação. 

E o `SelfBalancingRobot` funciona como orquestrador:

```text
SelfBalancingRobot
 ├── Mpu6050Sensor
 ├── KalmanFilter
 ├── BalanceController
 ├── PowerRamp
 └── MotorDriver
```

Isso está muito bom. Cada classe tem uma responsabilidade clara. 

Também gostei de você ter concentrado parâmetros de controle, MPU e temporização no `Config.h`. 

Eu **não desmontaria essa arquitetura**. Corrigiria o que cada bloco está fazendo.

---

# 2. O maior problema: você não mede inclinação

Atualmente o MPU6050 está lendo apenas:

```cpp
int16_t gx = ...
lastGyroX = correctedGyroX / gyroLsbPerDps();
```

Logo, `sensor.gyroX()` devolve **velocidade angular em graus por segundo**. 

Então seu fluxo atual é:

```text
MPU6050
   ↓
gyroX
   ↓
°/s
   ↓
Kalman
   ↓
PID
   ↓
motor
```

O problema fundamental é:

> **velocidade angular não é ângulo.**

Imagine:

```text
       carrinho
          /
         /
        O
```

Ele está inclinado `15°`.

Mas, durante um pequeno instante, não está girando.

Então:

```text
ângulo = 15°
gyroX ≈ 0°/s
```

Seu código recebe:

```cpp
rawGyroX = 0;
filteredGyroX = 0;
```

e depois:

```cpp
controller.compute(0);
```

Como seu setpoint é `0`, o PID pensa:

```text
erro = 0 - 0 = 0

"PERFEITO 👍"
```

enquanto o robô cai lindamente no chão.

O problema aparece claramente no `SelfBalancingRobot`:

```cpp
double rawGyroX = sensor.gyroX();
double filteredGyroX = filter.update(rawGyroX);
double controlSignal = controller.compute(filteredGyroX);
```



Esse é o primeiro ponto que precisamos corrigir **antes até de pensar em ajustar Kp/Ki/Kd**.

---

# 3. Precisamos usar acelerômetro + giroscópio

O MPU6050 tem exatamente os dois sensores que precisamos.

O giroscópio nos dá:

```text
velocidade angular
°/s
```

e podemos integrar:

```text
ângulo += gyro × dt
```

O problema é que qualquer pequeno erro do giroscópio também vai sendo integrado.

Por exemplo:

```text
erro = 0,2°/s

depois de 10s → 2°
depois de 60s → 12°
```

É o famoso drift.

O acelerômetro, por outro lado, consegue estimar a direção da gravidade.

Dependendo da orientação física do MPU, algo como:

```cpp
angleAccel = atan2(ay, az) * 180.0 / PI;
```

nos dá uma estimativa do ângulo.

Então:

```text
Gyro
 └── muito bom para movimentos rápidos
     mas deriva

Acelerômetro
 └── referência absoluta de gravidade
     mas é ruidoso e sente aceleração do carrinho
```

Juntamos os dois:

```text
        giroscópio ───┐
                      ├──> estimativa do ângulo
        acelerômetro ─┘
```

E **esse ângulo** é o que deve entrar no controlador.

---

# 4. Seu "Kalman" atual não faz essa fusão

Sua classe `KalmanFilter` está matematicamente coerente como um filtro escalar simples.

Ela recebe uma medida:

```cpp
double KalmanFilter::update(double measurement)
```

e assume essencialmente:

```text
estado atual ≈ estado anterior
```

depois corrige a estimativa com a nova medição usando o ganho de Kalman. 

Isso serve como um **suavizador da leitura do giroscópio**.

Mas não é o Kalman normalmente utilizado para estimar orientação de um robô, porque não existe:

```text
ângulo
velocidade angular
bias do giroscópio
acelerômetro
dt
```

no modelo.

Você está fazendo:

```text
gyroX → Kalman → gyroX menos ruidoso
```

quando precisamos de algo conceitualmente parecido com:

```text
Accel ─────────────┐
                   ├─→ estimador → ANGLE
Gyro ──────────────┘
```

---

# 5. E o filtro atual está bastante agressivo

Você configurou:

```cpp
KALMAN_Q = 0.0001;
KALMAN_R = 0.007;
```



Fazendo as contas para esse Kalman escalar, depois que ele converge o ganho fica aproximadamente:

```text
K ≈ 0,113
```

Então cada atualização fica aproximadamente:

```text
nova estimativa =
    88,7% estimativa anterior
  + 11,3% nova leitura
```

A 100 Hz, isso equivale aproximadamente a uma constante de tempo de **84 ms**.

Para alguma telemetria:

> suave, bonito.

Para um robô caindo:

> 84 ms é uma eternidade kkkkk.

Portanto existe potencial de introduzir atraso relevante justamente no sinal que deveria reagir rápido.

Não tentaria ajustar esses `Q` e `R` ainda. **Eu substituiria o modelo.**

---

# 6. Eu começaria com filtro complementar, não Kalman

Para o primeiro robô funcionando, eu faria:

```cpp
angle =
    alpha * (angle + gyroRate * dt)
    + (1.0 - alpha) * accelAngle;
```

Algo conceitualmente assim:

```text
                    ┌─────────────────────┐
gyro ── integração ─┤                     │
                    │ filtro complementar ├── ANGLE
accel ──────────────┤                     │
                    └─────────────────────┘
```

É muito mais simples de depurar.

Depois, quando tivermos o carrinho funcionando, podemos implementar um Kalman de estado:

```text
x = [
    angle
    gyro_bias
]
```

Isso sim seria um uso muito mais interessante do Kalman no projeto.

---

# 7. Existe outro bug importante: você implementou calibração, mas nunca chama

Você tem:

```cpp
void Mpu6050Sensor::calibrate(uint16_t samples)
```

e calcula:

```cpp
gyroXOffset = sumGyroX / samples;
```



Só que no `begin()` do robô você faz:

```cpp
motors.begin();
sensor.begin();

controller.begin(...);
```

e não existe:

```cpp
sensor.calibrate(...);
```



Então:

```cpp
long gyroXOffset = 0;
```

permanece zero. 

Isso significa que qualquer bias natural do MPU entra diretamente no controlador.

Eu faria, inicialmente:

```cpp
motors.begin();

if (!sensor.begin()) {
    motors.stop();
    return;
}

delay(1000);

sensor.calibrate(500);

controller.begin(...);
```

E o robô precisa ficar parado durante essa calibração.

---

# 8. Tem um pequeno erro na própria calibração

Você acumula somente leituras bem-sucedidas:

```cpp
if (readRegs(...)) {
    ...
    sumGyroX += gx;
}
```

mas depois sempre divide por:

```cpp
samples
```



Se pedir 500 amostras e 100 falharem:

```text
soma de 400 leituras
÷
500
```

O offset fica errado.

Precisamos de:

```cpp
uint16_t validSamples = 0;
```

e dividir por `validSamples`.

---

# 9. Outro problema sério: erros do sensor são ignorados

`Mpu6050Sensor::read()` pode retornar `false`:

```cpp
if (!readRegs(...)) {
    ...
    return false;
}
```



Mas o robô simplesmente faz:

```cpp
sensor.read();

double rawGyroX = sensor.gyroX();
```



Então se o I²C falhar:

```text
sensor perdeu comunicação
        ↓
mantém leitura anterior
        ↓
PID continua rodando
        ↓
motor continua obedecendo dado velho
```

Num display isso seria aceitável.

Num robô autobalanceado, não.

Precisa ser algo como:

```cpp
if (!sensor.read()) {
    motors.stop();
    return;
}
```

Idealmente também resetamos o estado do controlador/filtro depois de uma falha prolongada.

---

# 10. O teste `WHO_AM_I` também é fraco

Seu `begin()` faz:

```cpp
uint8_t who = readReg(WHO_AM_I);

return who != 0xFF;
```



Então qualquer coisa como:

```text
0x00
0x12
0x42
0xAB
```

é considerada MPU válido.

Para teste tudo bem, mas posteriormente convém validar realmente o dispositivo esperado.

---

# 11. Agora chegamos ao MotorDriver

A estrutura está legal.

Você abstraiu cada motor através de:

```cpp
struct MotorPort {
    int forwardPin;
    int backwardPin;
    int enablePin;
    ledc_channel_t pwmChannel;
    int pwmOffset;
};
```



Temos:

```text
RIGHT
IN1 = 26
IN2 = 25
ENA = 14

LEFT
IN4 = 32
IN3 = 33
ENB = 16
```



Note que você deliberadamente trocou `IN3/IN4` no motor esquerdo:

```cpp
RIGHT:
forward = IN1
backward = IN2

LEFT:
forward = IN4
backward = IN3
```

Isso faz sentido se os dois motores estão montados espelhados e você confirmou que:

```cpp
driveForward()
```

faz fisicamente as duas rodas levarem o carrinho na mesma direção.

Então essa parte está boa.

---

# 12. PWM também está implementado corretamente

Você configurou:

```cpp
PWM_FREQ = 5000;
PWM_RESOLUTION = 9 bits;
PWM_MAX_DUTY = 511;
```



E criou dois canais:

```text
RIGHT → LEDC_CHANNEL_0
LEFT  → LEDC_CHANNEL_1
```

compartilhando o timer 0. 

Isso está perfeitamente coerente para dois motores usando mesma frequência/resolução.

`5 kHz` funciona.

Posteriormente podemos experimentar uma frequência maior para tirar o PWM da região mais audível, mas isso **não é prioridade agora**.

---

# 13. O offset de 370 me preocupa MUITO

Você definiu:

```cpp
RIGHT_PWM_OFFSET = 370;
LEFT_PWM_OFFSET = 370;
```

e:

```cpp
int duty = constrain(
    motor.pwmOffset + power,
    0,
    PWM_MAX_DUTY
);
```

 

Seu máximo é:

```text
511
```

Então:

```text
370 / 511 = 72,4%
```

Significa que seu motor efetivamente tem:

```text
STOP
0%

↓ comando aparece ↓

~73%

74%
75%
...
100%
```

Você não utiliza:

```text
1% ───────────── 72%
```

Isso talvez esteja compensando exatamente o problema que encontramos com o L298N.

Mas existe uma conclusão importante:

> **se o motor realmente só consegue fornecer torque útil a partir de ~72% de PWM, nosso hardware possui pouquíssima autoridade de controle perto do equilíbrio.**

Para um carrinho autobalanceado queremos algo parecido com:

```text
pequeno erro → pequeno torque
erro maior   → torque maior
```

Seu sistema atual fica mais próximo de:

```text
pequeno erro → nada
um pouquinho maior → PÁ, 73%
```

Essa é uma receita maravilhosa para:

```text
esquerda
DIREITA
ESQUERDA
DIREITA
ESQUERDA
CHÃO
```

---

# 14. A zona morta piora esse salto

Você configurou:

```cpp
DEAD_BAND = 5.0;
```



E:

```cpp
if (fabs(controlSignal) <= DEAD_BAND)
    stop();
```



Portanto temos uma descontinuidade:

```text
controle = 4.99
motor = 0%

controle = 5.01
motor ≈ 73%+
```

Eu não usaria uma deadband grande no **erro/controlador de equilíbrio**.

Existe diferença entre:

```text
deadband do controlador
```

e

```text
PWM mínimo necessário para vencer atrito do motor
```

São problemas diferentes e devem ser tratados separadamente.

---

# 15. PowerRamp: eu removeria do loop de equilíbrio

Sua ideia é limitar a variação:

```cpp
delta = targetPower - currentPower;
delta = constrain(delta, -stepLimit, stepLimit);
currentPower += delta;
```



Com:

```text
MAX_STEP = 5
MAX_POWER = 140
período = 10 ms
```



Para ir:

```text
0 → 140
```

leva:

```text
140 / 5 = 28 ciclos

28 × 10ms = 280ms
```

**280 ms é uma quantidade enorme de atraso para um robô que está caindo.**

O controlador fala:

> PRECISO DE TORQUE AGORA

e o `PowerRamp` responde:

> claro chefe, vou aumentando devagarzinho 👍

Não queremos isso.

---

# 16. Mas existe um problema ainda pior no PowerRamp

Você suaviza somente:

```cpp
fabs(controlSignal)
```



Portanto o `PowerRamp` **não conhece o sentido**.

Imagine:

```text
controle anterior = +100
potência = 100
```

No próximo ciclo:

```text
controle = -100
```

Então:

```cpp
targetPower = fabs(-100);
```

vira:

```text
100
```

O PowerRamp pensa:

```text
anterior = 100
novo = 100
delta = 0

"não mudou nada"
```

Mas o `MotorDriver` olha o sinal:

```cpp
controlSignal >= 0
```

e inverte a direção. 

Resultado:

```text
100 para frente
       ↓
10ms depois
       ↓
100 para trás
```

sem passar por zero.

Isso é ruim tanto para controle quanto mecanicamente/eletricamente.

Esse é um **bug importante de projeto**, não apenas ajuste fino.

---

# 17. Para balanceamento, eu usaria comando assinado

Internamente eu faria o sistema inteiro trabalhar com algo parecido:

```text
motorCommand:

-511 <──────── 0 ────────> +511

 trás        parado        frente
```

Então:

```cpp
double u = controller.compute(...);
```

já representa:

```text
sinal → direção
magnitude → torque
```

E só no `MotorDriver` fazemos:

```cpp
if (u > 0) {
    frente
} else {
    trás
}
```

Se futuramente quisermos slew-rate limiting, fazemos no **valor assinado**:

```text
+100
+80
+60
+40
+20
  0
-20
-40
...
```

Nunca:

```text
+100 → -100
```

instantaneamente.

Mas para a primeira versão autobalanceando, eu sequer colocaria ramp.

**PID direto no motor.**

Menos variáveis para depurar.

---

# 18. O PID em si está bem encapsulado

A classe:

```cpp
BalanceController
```

está limpa.

Você define:

```cpp
SetOutputLimits(-maxPower, maxPower);
SetSampleTime(sampleTimeMs);
```



E consegue alterar:

```cpp
setSetpoint()
setTunings()
```



Isso é bom.

Eu manteria essa classe.

O problema não é o PID.

O problema é **o que você está entregando ao PID**.

---

# 19. Seus ganhos atuais basicamente formam um P

Hoje:

```cpp
KP = 0.8
KI = 0
KD = 0
```



Então você tem:

```text
P puro
```

Isso é perfeitamente aceitável para começar testes.

Mas **não vale a pena tunar agora**, porque a variável controlada está errada.

Quando passarmos a controlar ângulo:

```text
erro = setpointAngle - measuredAngle
```

aí sim começaremos:

```text
Ki = 0
Kd = 0
```

subindo `Kp`.

Depois introduzimos `Kd`.

`Ki` geralmente entra por último e talvez seja bem pequeno.

---

# 20. Existe também uma decisão sobre o D

Como já temos o giroscópio nos dando diretamente:

```text
velocidade angular
```

podemos inclusive fazer um controlador PD explicitamente:

```cpp
error = targetAngle - angle;

control =
    Kp * error
    - Kd * gyroRate;
```

Isso é extremamente interessante para robô autobalanceado.

Em vez de numericamente derivar o ângulo:

```text
angle[n] - angle[n-1]
```

já temos um sensor físico medindo exatamente a velocidade angular.

Eu provavelmente seguiria por esse caminho.

---

# 21. Frequência do loop: 100 Hz

Você usa:

```cpp
CONTROL_PERIOD_MS = 10;
```

logo:

```text
1000 / 10 = 100 Hz
```



E controla usando:

```cpp
millis()
```



**100 Hz não é absurdo.** Dá para fazer um robô lento funcionar.

Mas para esse tipo de aplicação eu miraria inicialmente em algo como:

```text
200–250 Hz
```

e só subiria além se necessário.

Algo como:

```text
4 ms → 250 Hz
```

Isso reduz bastante a latência.

---

# 22. Para fusão do sensor precisamos abandonar `millis()` como referência de integração

Para:

```cpp
angle += gyro * dt;
```

precisamos conhecer `dt` razoavelmente bem.

Então eu usaria:

```cpp
micros()
```

e calcularia:

```cpp
float dt =
    (nowMicros - previousMicros) * 1e-6f;
```

Assim, mesmo se um ciclo durar:

```text
3.97 ms
```

e outro:

```text
4.13 ms
```

a integração continua correta.

---

# 23. O Serial dentro do loop também merece cuidado

Você imprime em toda atualização:

```cpp
Serial.printf(
    "%d\t%d\t%d\t%lf\t%lf\n",
    ...
);
```



Para depuração isso é ótimo.

No controle final, não imprimiria em todo ciclo.

Por exemplo:

```text
controle: 250 Hz
telemetria: 20 Hz
```

Podemos fazer:

```cpp
if (now - lastTelemetry >= 50) {
    Serial.printf(...);
}
```

Assim a comunicação serial não interfere tanto no timing.

---

# 24. I²C a 100 kHz também pode ser melhorado depois

Hoje:

```cpp
I2C_CLOCK = 100000;
```



Não há nada errado nisso.

Mas estamos lendo 14 bytes por ciclo. 

Quando passarmos o loop para frequências maiores, podemos usar:

```text
400 kHz
```

se o barramento e o módulo estiverem estáveis.

Não é prioridade agora.

---

# 25. Falta um sistema de segurança

Hoje não existe conceito de:

```text
robô caiu
```

Isso é perigoso.

Imagine o controlador saturado:

```text
ângulo = 75°
PWM = máximo
```

Nesse ponto não existe mais chance realista do robô recuperar equilíbrio.

O resultado é ele ficar no chão com:

```text
MOTORRRRRRRRRRRRRRRR
```

Eu colocaria:

```cpp
if (fabs(angle) > MAX_SAFE_ANGLE) {
    motors.stop();
}
```

Algo inicialmente próximo de:

```text
30–40°
```

e ajustamos depois.

Também precisaremos de estado:

```text
DISARMED
CALIBRATING
READY
BALANCING
FALLEN
ERROR
```

Não precisa virar Enterprise Java embarcado™, mas alguma máquina de estados simples vai ajudar muito.

---

# 26. Também falta failsafe de sensor

Eu consideraria estes casos fatais:

```text
MPU não inicializou
MPU parou de responder
leitura ficou inválida
ângulo impossível
```

Resposta:

```text
PWM = 0
motores = STOP
```

Nada de tentar ser herói com dado velho.

---

# 27. Há ainda uma questão física: eixo do MPU

Seu código assume:

```cpp
gyroX
```



Isso só está correto se o eixo **X físico do MPU** for justamente o eixo em torno do qual o robô cai para frente/trás.

Precisamos testar isso.

Com o carrinho parado e monitor serial:

```text
inclina para frente
```

somente um dos:

```text
gyroX
gyroY
gyroZ
```

deve responder fortemente.

Esse é o eixo que devemos usar.

Não assumiria X só porque o código começou com X.

---

# 28. E o setpoint provavelmente não será exatamente zero

Você colocou:

```cpp
SETPOINT = 0.0;
```



Perfeito para começar.

Mas fisicamente o ponto de equilíbrio pode acabar sendo:

```text
+1.4°
```

ou:

```text
-2.1°
```

por causa de:

```text
posição da bateria
motor
estrutura
posição do MPU
distribuição de massa
```

Então o setpoint precisa permanecer configurável, como já está.

---

# 29. O L298N continua sendo nosso elefante na sala

Seu código:

```cpp
PWM_OFFSET = 370;
```

me parece muito provavelmente uma tentativa de compensar:

```text
L298N + 5V + motor pequeno
```

que já vimos gerar somente uns `3 V` efetivos no motor.

Isso significa que software está tentando corrigir uma limitação de hardware.

Até certo ponto funciona.

Mas se realmente constatarmos:

```text
abaixo de 370/511:
motor praticamente inútil
```

eu trocaria o driver.

Porque um PID precisa modular torque de maneira relativamente fina:

```text
10%
15%
20%
25%
...
```

Se nosso atuador só responde em:

```text
0%
73–100%
```

estamos tentando jogar tênis usando um martelo.

O DRV8833 ou TB6612FNG que discutimos antes faria bastante sentido aqui.

---

# 30. Também precisamos medir os motores

Antes da sintonia final eu quero descobrir empiricamente, para **cada motor**:

```text
menor PWM que começa a girar
menor PWM que continua girando
PWM máximo desejado
diferença entre esquerda/direita
```

Você já preparou a arquitetura para isso:

```cpp
RIGHT_PWM_OFFSET
LEFT_PWM_OFFSET
```



Isso é bom.

Talvez descubramos:

```text
direito: 135
esquerdo: 147
```

em vez de os dois `370`.

Esses números precisam vir de medição, não de chute.

---

# 31. Sem encoder ele pode se equilibrar, mas vai andar

Outro detalhe importante do projeto inteiro:

não existe encoder.

Isso **não impede o primeiro autobalanceamento**.

Podemos fazer:

```text
IMU
 ↓
controle de ângulo
 ↓
motores
```

e ele se manter aproximadamente ereto.

Mas nada diz ao controlador:

```text
você saiu 1 metro do lugar
```

Então ele pode:

```text
ficar em pé
↓
começar a andar
↓
continuar corrigindo o próprio equilíbrio
↓
ir embora pela sala
```

kkkk.

A arquitetura mais completa depois seria:

```text
         ENCODERS
            ↓
     controle posição/
        velocidade
            ↓
       angle target
            ↓
           PID
         de ângulo
            ↓
          MOTOR
            ↑
            │
        MPU6050
```

Ou seja:

```text
loop externo → posição/velocidade
loop interno → equilíbrio
```

Mas **não coloque encoder agora**. Primeiro fazemos o inner loop funcionar.

---

# Minha ordem de ataque

Eu faria exatamente nesta sequência:

1. **Modificar `Mpu6050Sensor` para ler acelerômetro + giroscópio.**
2. Fazer calibração do gyro obrigatoriamente no boot.
3. Produzir `accelAngle`, `gyroRate` e `angle`.
4. Substituir temporariamente o Kalman por um filtro complementar.
5. Fazer o controlador receber **ângulo**, não `gyroX`.
6. Remover `PowerRamp` do balanceamento.
7. Fazer todo o comando do motor trabalhar assinado.
8. Adicionar parada por ângulo excessivo e falha do MPU.
9. Reduzir o período para aproximadamente `4–5 ms`.
10. Só então começar a sintonia de `Kp/Kd/Ki`.

Eu **não mexeria nos ganhos do PID antes dos passos 1–8**. Estaríamos tentando tunar um controlador cuja variável de entrada não representa aquilo que queremos controlar.

## Como eu quero que o fluxo fique

Hoje:

```text
gyroX
  ↓
Kalman simples
  ↓
PID
  ↓
abs()
  ↓
PowerRamp
  ↓
offset 370
  ↓
L298N
```

Eu transformaria em:

```text
                 ┌── acelerômetro
MPU6050 ─────────┤
                 └── giroscópio
                         │
                         ▼
                FUSÃO DE SENSORES
                         │
                 ┌───────┴───────┐
                 │               │
               angle         gyroRate
                 │               │
                 └───────┬───────┘
                         ▼
                    PD / PID
                         │
                  comando assinado
                    -MAX ... +MAX
                         │
                         ▼
                    MotorDriver
                         │
                  ┌──────┴──────┐
                  ▼             ▼
              motor L        motor R
```

Esse é o projeto que tem condições reais de se equilibrar.

E tem uma notícia boa no meio dos problemas: **não acho que seu projeto esteja “ruim” e precise ser refeito.** A divisão das classes já está adequada. O que precisa mudar é principalmente o conteúdo de `Mpu6050Sensor`, o papel do `KalmanFilter`, o sinal entregue ao `BalanceController` e a forma como `PowerRamp/MotorDriver` tratam o comando. A espinha dorsal pode ficar exatamente onde está. 

O próximo passo que eu faria é **reescrever conosco somente o bloco do MPU6050**, adicionando `accelAngle`, `gyroRate`, calibração e cálculo de `dt`, sem tocar ainda no PID. Aí conseguimos testar cada informação no Serial antes de sequer energizar os motores.
