---
layout: math
title: Filtro de Kalman em 1D
permalink: /filtro_de_kalman_1d.html
---

# Filtro de Kalman em uma dimensão (1D)

Para estruturar o Filtro de Kalman exatamente como ele opera matematicamente, vamos utilizar um **modelo escalar (1D)**. Essa abordagem unidimensional (por exemplo, rastrear a posição de um objeto em uma linha reta) permite visualizar cada multiplicação, soma e inversão de forma cristalina, sem a complexidade de matrizes multidimensionais, mantendo toda a rigorosidade do algoritmo original.

## Cenário e Parâmetros Iniciais (Instante $$k = 0$$)

Imagine que estamos rastreando a posição de um objeto (em metros). No instante anterior, temos os seguintes dados base:

- **$$\hat{x}_{0} = 10.0$$**: Estimativa anterior da posição (em metros).
- **$$P_{0} = 2.0$$**: Incerteza inicial (variância do erro da estimativa). Quanto maior, menos confiável.
- **$$F = 1$$**: Matriz de transição de estado (assume que o objeto está parado ou com velocidade constante previsível).
- **$$H = 1$$**: Matriz de observação (o sensor mede diretamente a posição na mesma escala).
- **$$Q = 0.1$$**: Ruído do processo (incerteza adicionada pelo modelo físico a cada passo, ex: vibrações).
- **$$R = 1.0$$**: Ruído da medição (a imprecisão inerente do sensor/GPS).

Agora, o tempo avança para o instante **$$k = 1$$**. O sensor nos fornece uma nova leitura: **$$z_1 = 10.5$$ metros**.

Vamos executar os **5 passos matemáticos oficiais** do Filtro de Kalman com esses números.

---

## Fase 1: Previsão (Time Update)

Nesta fase, o sistema projeta o que a física e o modelo esperam que aconteça antes de olhar para o sensor.

### Passo 1: Previsão do Estado ($$\hat{x}_k^-$$)

O modelo calcula onde o objeto deveria estar com base no estado anterior.

**Fórmula:**

$$
\hat{x}_k^- = F \cdot \hat{x}_{k-1}
$$

**Substituição e cálculo:**

$$
\hat{x}_1^- = 1 \cdot 10.0 = 10.0 \text{ metros}
$$

**Significado:** pelo nosso modelo, o objeto continua na posição $$10.0$$.

### Passo 2: Previsão da Covariância do Erro ($$P_k^-$$)

O modelo calcula como a incerteza cresce devido às imperfeições do mundo real ($$Q$$).

**Fórmula:**

$$
P_k^- = F \cdot P_{k-1} \cdot F^T + Q
$$

**Substituição e cálculo:**

$$
P_1^- = (1) \cdot (2.0) \cdot (1) + 0.1 = 2.0 + 0.1 = 2.1
$$

**Significado:** como o tempo passou e o modelo não é perfeito, nossa incerteza cresceu de $$2.0$$ para **$$2.1$$**.

---

## Fase 2: Atualização / Correção (Measurement Update)

O sensor entrega a nova medição ($$z_1 = 10.5$$). O filtro agora combina a previsão com a nova evidência.

### Passo 3: Cálculo do Ganho de Kalman ($$K_k$$)

O Ganho de Kalman determina o peso relativo entre a nossa previsão e o sensor, comparando as incertezas. Em formato escalar, a fórmula é:

**Fórmula:**

$$
K_k = \frac{P_k^- \cdot H}{H \cdot P_k^- \cdot H + R}
$$

**Substituição e cálculo:**

$$
K_1 = \frac{2.1 \cdot 1}{(1) \cdot (2.1) \cdot (1) + 1.0} = \frac{2.1}{2.1 + 1.0} = \frac{2.1}{3.1} \approx 0.677
$$

**Significado:** o Ganho de Kalman é de aproximadamente **$$0.677$$**. Isso significa que daremos $$67.7\%$$ de peso para o sensor e $$32.3\%$$ para o modelo físico (o sensor ganha mais peso porque sua incerteza $$R = 1.0$$ é menor que a incerteza da nossa previsão $$P_1^- = 2.1$$).

### Passo 4: Atualização do Estado ($$\hat{x}_k$$)

Ajustamos a posição prevista somando o erro da medição (a diferença entre o que o sensor viu e o que esperávamos) ponderado pelo Ganho de Kalman.

**Fórmula:**

$$
\hat{x}_k = \hat{x}_k^- + K_k \cdot (z_k - H \cdot \hat{x}_k^-)
$$

**Substituição e cálculo:**

- Erro de medição (inovação): $$10.5 - (1 \cdot 10.0) = 10.5 - 10.0 = +0.5$$
- Ajuste: $$0.677 \cdot 0.5 \approx 0.3385$$
- Estimativa: $$\hat{x}_1 = 10.0 + 0.3385 = \mathbf{10.3385 \text{ metros}}$$

**Significado:** nossa estimativa final corrigida para o instante $$1$$. O sensor dizia $$10.5$$, o modelo previa $$10.0$$, e o filtro encontrou o meio-termo ideal em **$$10.34$$ metros**.

### Passo 5: Atualização da Covariância do Erro ($$P_k$$)

Por fim, recalculamos a nossa incerteza. Como obtivemos dados novos de um sensor confiável, nossa dúvida deve encolher.

**Fórmula:**

$$
P_k = (1 - K_k \cdot H) \cdot P_k^-
$$

**Substituição e cálculo:**

$$
P_1 = (1 - 0.677 \cdot 1) \cdot 2.1 = (0.323) \cdot 2.1 \approx \mathbf{0.678}
$$

**Significado:** nossa incerteza despencou de $$2.1$$ para **$$0.678$$**.

---

## O Ciclo se Repete

Agora, para o próximo instante de tempo ($$k = 2$$):

1. O novo estado ($$\hat{x}_1 = 10.3385$$) e a nova covariância ($$P_1 = 0.678$$) tornam-se os valores iniciais do próximo ciclo.
2. O algoritmo volta para o **Passo 1 (Previsão)**.

Veja também o [algoritmo padrão]({{ '/filtro_de_kalman.html' | relative_url }}) e o [exemplo em 2D]({{ '/filtro_de_kalman_2d.html' | relative_url }}).
