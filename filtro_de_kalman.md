---
layout: default
title: Filtro de Kalman (Algoritmo Padrão)
permalink: /filtro_de_kalman.html
---

# Filtro de Kalman (Algoritmo Padrão)

O Filtro de Kalman é um estimador recursivo ideal para sistemas lineares afetados por ruído gaussiano. O algoritmo opera em um ciclo contínuo dividido em duas grandes fases: **Previsão (Time Update)** e **Atualização (Measurement Update)**.

Abaixo está o guia passo a passo generalizado com todas as fórmulas matemáticas necessárias para implementar o filtro em qualquer dimensão (1D, 2D ou multidimensional).

---

## 0. Glossário de Símbolos

* $\hat{x}_{k-1}$: Vetor de estado estimado no instante anterior ($k-1$).
* $P_{k-1}$: Matriz de covariância do erro (incerteza) no instante anterior.
* $F$: Matriz de transição de estado (modelo físico do sistema).
* $Q$: Matriz de covariância do ruído do processo (imperfeições do modelo).
* $H$: Matriz de observação (mapeia o espaço de estado para o espaço de medição do sensor).
* $R$: Matriz de covariância do ruído da medição (imprecisão do sensor).
* $z_k$: Vetor de medição obtido pelo sensor no instante atual ($k$).
* $K_k$: Ganho de Kalman no instante atual.
* $I$: Matriz identidade.

---

## Fase 1: Previsão (Time Update)
Nesta fase, o sistema projeta o estado futuro e a incerteza com base puramente no modelo físico, sem consultar os sensores ainda.

### Passo 1: Previsão do Estado a Priori ($\hat{x}_k^-$)
Projeta a estimativa do estado para o instante atual utilizando o modelo físico do sistema.

$$
\hat{x}_k^- = F \cdot \hat{x}_{k-1}
$$

### Passo 2: Previsão da Covariância do Erro a Priori ($P_k^-$)
Projeta a incerteza do sistema para o instante atual, acumulando a incerteza anterior propagada pelo modelo e somando o ruído do processo.

$$
P_k^- = F \cdot P_{k-1} \cdot F^T + Q
$$

---

## Fase 2: Atualização / Correção (Measurement Update)
Nesta fase, o sensor fornece uma nova leitura ($z_k$). O algoritmo compara a medição com a previsão e recalibra tanto o estado quanto a incerteza.

### Passo 3: Cálculo do Ganho de Kalman ($K_k$)
Calcula o fator de ponderação ideal que decide o equilíbrio entre a confiança na previsão do modelo e a confiança na medição do sensor.

$$
K_k = P_k^- \cdot H^T \cdot (H \cdot P_k^- \cdot H^T + R)^{-1}
$$

### Passo 4: Atualização do Estado a Posteriori ($\hat{x}_k$)
Ajusta a estimativa prevista somando o erro residual (a diferença entre a medição real e o que o modelo esperava ver, chamada de *inovação*), ponderado pelo Ganho de Kalman.

$$
\hat{x}_k = \hat{x}_k^- + K_k \cdot (z_k - H \cdot \hat{x}_k^-)
$$

### Passo 5: Atualização da Covariância do Erro a Posteriori ($P_k$)
Atualiza (reduz) a matriz de incerteza do sistema, uma vez que a incorporação de uma nova medição confiável diminui a nossa dúvida geral sobre o estado.

$$
P_k = (I - K_k \cdot H) \cdot P_k^-
$$

---

## O Ciclo Contínuo
Após concluir o **Passo 5**, os valores atualizados ($\hat{x}_k$ e $P_k$) tornam-se a base ($\hat{x}_{k-1}$ e $P_{k-1}$) para o próximo ciclo de tempo ($k+1$).