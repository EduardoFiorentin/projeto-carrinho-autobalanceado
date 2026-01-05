Excelente código. Dá pra ver que **não é projeto amador**.
Agora: **dá pra reduzir bastante os resets só mexendo no software**, sim.
Vou analisar **linha de raciocínio por linha de impacto elétrico**, não estilo.

Vou dividir por **problema → por que causa reset → o que mudar no código**.

---

# 🔥 PROBLEMA 1 — PWM muda rápido demais (pico de corrente)

## O que seu código faz hoje

A cada ciclo de controle (5 ms):

```cpp
pid.Compute();
power = constrain(abs(pid_out), 0, 140);
set_duty(... LEFT_ACT_PRESET + power ...)
```

Ou seja:

* `pid_out` muda abruptamente
* `power` muda abruptamente
* PWM muda **instantaneamente**

Isso gera:

* pico de corrente no motor
* queda de tensão
* reset no ESP

⚠️ **Esse é o principal gatilho elétrico via software.**

---

## ✅ Correção: rampa de potência (OBRIGATÓRIO)

Você **NUNCA** deve aplicar o PID direto no PWM.

Crie uma variável de potência aplicada:

```cpp
int applied_power = 0;
```

E limite a taxa de variação:

```cpp
const int MAX_POWER_STEP = 5; // ajuste fino depois
```

Substitua isso:

```cpp
power = constrain(abs(pid_out), 0, 140);
```

Por isso:

```cpp
int target_power = constrain(abs(pid_out), 0, 140);

if (applied_power < target_power)
  applied_power = min(applied_power + MAX_POWER_STEP, target_power);
else if (applied_power > target_power)
  applied_power = max(applied_power - MAX_POWER_STEP, target_power);
```

E use **applied_power** no PWM.

🎯 Resultado:

* corrente sobe devagar
* ruído cai brutalmente
* reset quase some

---

# ⚡ PROBLEMA 2 — Inversão instantânea de sentido

## O que acontece hoje

Quando `pid_out` troca de sinal:

```cpp
if (pid_out >= 0) { ... }
else { ... }
```

Você:

* desliga um lado
* liga o outro **no mesmo ciclo**

Isso é:

* curto momentâneo dentro da ponte H
* pico absurdo de corrente
* back-EMF violento

---

## ✅ Correção: dead-time de inversão (crítico)

Implemente um **estado neutro** antes de inverter.

Crie:

```cpp
enum MotorDir { STOPPED, FORWARD, BACKWARD };
MotorDir currentDir = STOPPED;
uint32_t lastDirChange = 0;
const uint32_t DIR_DEADTIME_MS = 10;
```

Antes de trocar sentido:

```cpp
MotorDir targetDir = pid_out >= 0 ? FORWARD : BACKWARD;

if (targetDir != currentDir) {
  // força parada
  set_duty(ledChannelE1, 0);
  set_duty(ledChannelE2, 0);
  set_duty(ledChannelD1, 0);
  set_duty(ledChannelD2, 0);

  if (millis() - lastDirChange >= DIR_DEADTIME_MS) {
    currentDir = targetDir;
    lastDirChange = millis();
  }
  return;
}
```

Só depois disso você aplica PWM.

🎯 Resultado:

* elimina shoot-through
* reduz EMI
* protege ponte e ESP

---

# 🧨 PROBLEMA 3 — Frequência de PWM relativamente alta

Você usa:

```cpp
const int freq = 5000;
```

5 kHz é ok **para motor**, mas:

* aumenta EMI
* piora retorno de ruído
* não traz ganho real em torque

---

## ✅ Teste recomendado

Teste **1 kHz ou 2 kHz**:

```cpp
const int freq = 2000;
```

Motor DC não precisa de 5 kHz.
PID agradece.
ESP também.

---

# 🕒 PROBLEMA 4 — Controle de tempo impreciso

Você faz:

```cpp
if (now - lastControl < CONTROL_PERIOD_MS) return;
lastControl = now;
```

Isso cria:

* jitter
* atraso acumulado
* variação no `dt` do PID

PID odeia `dt` inconsistente → saída mais agressiva → mais pico.

Boa. Esse é **o segundo maior vilão** depois do PWM nervoso.
Vamos abrir isso **até o osso**, usando **exatamente o seu código**.

---

# 📡 I2C lento + ruído = leituras erráticas

## (e como isso vira PID surtado + reset)

---

## 1️⃣ O que está acontecendo fisicamente

Você tem:

* motores DC (corrente alta)
* PWM comutando
* fios longos
* barramento I2C (SDA / SCL) **sensível pra caramba**

I2C:

* usa níveis lógicos analógicos
* depende de resistores de pull-up
* **não é imune a ruído**

Motor ligado = campo eletromagnético = lixo entrando no barramento.

---

## 2️⃣ O que significa “leitura errática”

Não é só “valor errado”.

Pode ser:

* leitura incompleta
* atraso na resposta
* byte corrompido
* clock esticado (clock stretching)
* travamento momentâneo do barramento

Tudo isso acontece **sem crash visível**.

---

## 3️⃣ Onde isso aparece no SEU código

### Aqui 👇

```cpp
if (!readRegs(ACCEL_XOUT_H, 14, buf)) {
  Serial.println("Error reading register value, retaining last value");
  return gyro_val;
}
```

Parece seguro, mas **não é suficiente**.

### Por quê?

Porque o problema mais comum **não é falhar a leitura**.
É **ler lixo válido**.

Exemplo real:

```text
gyro_x normal:   1.2
ruído I2C:     180.5  ← dado "válido", mas absurdo
```

O código:

* aceita
* passa pro Kalman
* PID entra em pânico
* PWM explode

---

## 4️⃣ Como isso vira pico elétrico

Cadeia exata:

```
Ruído no I2C →
Leitura absurda →
PID acha que está caindo →
PWM sobe tudo →
Motor puxa pico →
Tensão cai →
ESP reseta
```

Isso explica:

* reset “aleatório”
* reset só quando motor gira
* reset impossível de debugar no código

---

## 5️⃣ Wire + motores = atraso invisível

Você usa:

```cpp
Wire.setClock(100000);
```

100 kHz é robusto, MAS:

* ruído pode causar retry interno
* `Wire.requestFrom()` pode demorar mais
* o loop fica bloqueado esperando

Isso cria:

* atraso variável
* jitter no controle
* PID mais agressivo

⚠️ Mesmo sem erro explícito.

---

## 6️⃣ O erro conceitual mais comum

> “Mas eu já uso Kalman, ele filtra isso”

❌ **Não filtra ruído impulsivo grande**.

Kalman:

* assume ruído gaussiano pequeno
* não lida bem com outliers
* um pico grande passa direto

Ou pior: **contamina o estado interno**.

---

## 7️⃣ Solução correta (software) — em camadas

### 🛡️ Camada 1 — Sanidade do valor lido (OBRIGATÓRIO)

Antes de usar o valor:

```cpp
static float last_valid_gyro = 0;

float gx = read_gyroscope_x();

if (isnan(gx) || abs(gx) > 1500) {
  gyro_x = last_valid_gyro;
} else {
  gyro_x = gx;
  last_valid_gyro = gx;
}
```

Por quê `1500`?

* você configurou ±1000 °/s
* qualquer coisa próxima disso é lixo em carrinho

---

### 🛡️ Camada 2 — Limitar variação entre amostras

Ruído I2C geralmente aparece como **salto instantâneo**.

```cpp
const float MAX_GYRO_STEP = 50.0; // ajuste fino

float delta = gyro_x - last_valid_gyro;

if (abs(delta) > MAX_GYRO_STEP) {
  gyro_x = last_valid_gyro;
}
```

Isso mata:

* glitches
* leituras espúrias
* surtos no PID

---

### 🛡️ Camada 3 — Isolar leitura do controle

Hoje você faz tudo junto.

Faça mentalmente:

1. lê sensor
2. valida
3. filtra
4. só então controla

Nunca controle com dado cru.

---

## 8️⃣ Por que isso reduz resets (mesmo sendo “só software”)

Porque você impede que:

* uma leitura ruim
* cause uma decisão ruim
* que cause um pico elétrico

Menos decisões ruins → menos PWM agressivo → menos corrente.

---

## 9️⃣ Dica avançada (ouro puro)

Leia o MPU **em taxa maior** que o controle.

Exemplo:

* MPU a 1 kHz
* controle a 200 Hz

E use:

* média
* ou último valor válido

Isso desacopla:

* ruído do barramento
* do controle do motor


---

## ✅ Correção correta

Use soma incremental:

```cpp
if (now - lastControl >= CONTROL_PERIOD_MS) {
  lastControl += CONTROL_PERIOD_MS;
} else {
  return;
}
```

🎯 Resultado:

* período estável
* PID mais suave
* menos variação de potência

---

# 📢 PROBLEMA 6 — Serial.print dentro do loop de controle

Isso aqui é **vilão silencioso**:

```cpp
pr(gyro_x);
pr("\t");
pr(gyro_val);
pr("\t");
pln(pid_out);
```

Serial:

* bloqueia
* atrasa loop
* cria jitter
* aumenta agressividade do PID

---

## ✅ Correção simples

Logue **menos vezes**:

```cpp
static uint32_t lastLog = 0;
if (millis() - lastLog > 50) {
  lastLog = millis();
  Serial.printf("%f\t%f\t%f\n", gyro_x, gyro_val, pid_out);
}
```

🎯 Resultado:

* loop mais estável
* PWM menos nervoso

---

# 🧠 PROBLEMA 7 — PID sem banda morta

Pequeno ruído → motor se mexe → pico elétrico → reset.

---

## ✅ Banda morta

Antes de aplicar potência:

```cpp
if (abs(pid_out) < 2.0) {
  applied_power = 0;
}
```

Isso:

* elimina micro-oscilação
* reduz consumo
* melhora estabilidade

Perfeito. Vamos **esmiuçar a banda morta no PID** até não sobrar nenhuma dúvida.
Isso aqui é **controle clássico**, mas aplicado ao **mundo sujo dos motores reais**.

---

# 🧠 PROBLEMA 7 — PID **sem banda morta**

## Por que “quase zero” ainda é problema sério

---

## 1️⃣ O erro fundamental: achar que zero é um ponto, não uma faixa

No mundo matemático:

```
erro = 0
```

acabou o problema.

No mundo real:

* sensor tem ruído
* I2C tem glitch
* filtro tem atraso
* motor tem folga mecânica
* o carrinho vibra

Resultado:
👉 **erro nunca é zero**, ele **oscila em torno de zero**.

Exemplo realista do seu sistema:

```
+0.4, -0.6, +0.3, -0.5, +0.2...
```

Isso parece pequeno.
Mas para um PID **não é**.

---

## 2️⃣ O que o PID faz com erro pequeno

PID não pensa. Ele reage.

Mesmo erro minúsculo gera saída:

```
P = Kp * erro
```

Com seu `Kp = 4.0`:

```
erro = 0.5  → saída = 2
erro = -0.5 → saída = -2
```

Agora veja o efeito em cadeia:

```
erro muda de sinal →
PID muda de sinal →
motor troca sentido →
ponte H comuta →
pico de corrente →
ruído elétrico →
ESP sofre
```

Tudo isso **sem o carrinho sequer se mover de verdade**.

---

## 3️⃣ O fenômeno real: “chattering”

Esse comportamento tem nome:

👉 **Chattering**
(micro-oscilações rápidas ao redor do equilíbrio)

Características:

* motor fica “chiando”
* PWM fica ligando/desligando
* consumo sobe
* torque útil = zero
* ruído elétrico = máximo

É o pior cenário possível:
❌ não estabiliza
❌ não anda
❌ só gera problema elétrico

---

## 4️⃣ Por que isso piora resets no ESP

Agora junta com o hardware real:

* cada micro acionamento do motor
* gera um micro pico de corrente
* esses picos acontecem **o tempo todo**
* a alimentação nunca “descansa”

O ESP:

* tem regulador interno limitado
* detecta queda mínima de tensão
* **reseta**

Ou seja:

> não é um pico grande
> são **mil picos pequenos por segundo**

---

## 5️⃣ O erro comum: “mas meu PID já está estável”

Estável **matematicamente** ≠ estável **eletricamente**.

Você pode ter:

* gráfico lindo
* erro médio zero
* carrinho aparentemente parado

E ainda assim:

* ESP resetando
* MPU enlouquecendo
* ponte H aquecendo

---

## 6️⃣ O que é banda morta (deadband / dead zone)

### Conceito simples

Uma **faixa de erro** onde o controlador **não faz nada**.

Exemplo:

```
erro entre -ε e +ε → saída = 0
```

Visual:

```
        |
 saída  |        /
        |       /
        |______/
        |      \
        |       \
        |
        +----------------
              erro
```

No centro:

* silêncio
* motor desligado
* zero comutação
* zero pico

---

## 7️⃣ Banda morta aplicada AO SEU CÓDIGO

Hoje você faz:

```cpp
pid.Compute();
power = abs(pid_out);
```

Você deve fazer **antes** de aplicar potência:

```cpp
const double DEAD_BAND = 2.0;

if (abs(pid_out) < DEAD_BAND) {
  applied_power = 0;
  return;
}
```

Isso garante:

* motor realmente desligado
* ponte H quieta
* alimentação limpa

---

## 8️⃣ “Mas isso não cria erro estático?”

Excelente pergunta. Resposta honesta:

👉 **Sim, cria. E isso é bom.**

Em carrinho autobalanceado:

* você NÃO quer perseguir erro zero absoluto
* você quer **equilíbrio estável**

O erro residual:

* é absorvido pela mecânica
* pela inércia
* pela gravidade

Sem banda morta:

* você luta contra o ruído
* não contra a física real

---

## 9️⃣ Banda morta + Kalman + PID = casamento correto

Cada um tem papel diferente:

* **Kalman** → suaviza ruído pequeno
* **Banda morta** → ignora ruído residual
* **PID** → corrige erro real

Kalman **não substitui** banda morta.
Eles resolvem problemas diferentes.

---

## 🔥 Analogia brutal (mas perfeita)

Imagine segurar um lápis em pé na mão:

* se você corrigir cada micro tremida → ele cai
* se você ignorar tremidas pequenas → ele fica

Banda morta é **ignorar a tremida**.

---

## 10️⃣ Valores práticos (não chute)

Comece com:

```
DEAD_BAND = 1.0 a 3.0 (em °/s)
```

Ajuste observando:

* motor em silêncio no equilíbrio
* ESP parando de resetar
* temperatura da ponte H

---

## 🧾 Resumo sem dó

* erro nunca é zero na vida real
* PID sem banda morta comuta sem parar
* comutação = pico elétrico
* pico elétrico = reset
* banda morta resolve isso elegantemente

---

Se quiser, próximo nível:

* **banda morta adaptativa**
* **banda morta só no zero-cross**
* **PID híbrido (P fora, PD perto do equilíbrio)**

Agora você está entrando em **controle de verdade**, não tutorial de internet.


---

# ✅ CONCLUSÃO DIRETA (SEM ROMANCE)

Seu hardware **provavelmente já está no limite**.
Seu software hoje:

❌ muda PWM rápido
❌ inverte motor seco
❌ não respeita dead-time
❌ cria jitter no PID

Com as mudanças acima:

* consumo cai
* EMI despenca
* reset vira exceção

Se quiser, no próximo passo eu:

* **reestruturo seu loop inteiro**
* deixo ele em **estado-máquina**
* pronto pra carrinho autobalanceado sério

Você está **muito perto** de um sistema estável.
