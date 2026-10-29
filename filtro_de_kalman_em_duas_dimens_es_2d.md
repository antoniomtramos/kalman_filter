---
layout: default
title: Filtro de Kalman em 2D
permalink: /filtro_de_kalman_2d.html
---


# Filtro de Kalman para **duas dimensões (2D)** 

como rastrear a posição e a velocidade de um drone ou veículo em um plano cartesiano ($X$ e $Y$) — exige o uso de **vetores e matrizes**.

O modelo mais clássico para isso é o de **Velocidade Constante em 2D**, que monitora 4 variáveis (o estado):

1. Posição no eixo $X$ ($p_x$)

2. Posição no eixo $Y$ ($p_y$)

3. Velocidade no eixo $X$ ($v_x$)

4. Velocidade no eixo $Y$ ($v_y$)

Vamos percorrer cada passo com números reais para um intervalo de tempo $\Delta t = 1$ segundo.

### Parâmetros Iniciais e Matrizes (Instante $k = 0$)

* **Vetor de Estado Inicial (**$\mathbf{x}_0$**):** O objeto está na origem $(0,0)$ e se movendo a $1\text{ m/s}$ em $X$ e $2\text{ m/s}$ em $Y$.
  

  $$
  \mathbf{x}_0 = \begin{bmatrix} p_x \\ p_y \\ v_x \\ v_y \end{bmatrix} = \begin{bmatrix} 0.0 \\ 0.0 \\ 1.0 \\ 2.0 \end{bmatrix}
  $$

* **Matriz de Covariância Inicial (**$P_0$**):** Nossa incerteza inicial para todas as variáveis (assumida como $1$ na diagonal).
  

  $$
  P_0 = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}
  $$

* **Matriz de Transição de Estado (**$F$**):** Relaciona a física do movimento (posição atual = posição anterior + velocidade $\times \Delta t$).
  

  $$
  F = \begin{bmatrix} 1 & 0 & \Delta t & 0 \\ 0 & 1 & 0 & \Delta t \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix} = \begin{bmatrix} 1 & 0 & 1 & 0 \\ 0 & 1 & 0 & 1 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}
  $$

* **Matriz de Observação (**$H$**):** O sensor (ex: GPS) mede apenas a **posição** ($p_x$ e $p_y$), mas **não mede diretamente a velocidade**.
  

  $$
  H = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix}
  $$

* **Ruído do Processo (**$Q$**) e Ruído da Medição (**$R$**):**

  * $Q = 0.1 \cdot I_4$ (pequena instabilidade física no movimento).

  * $R = \begin{bmatrix} 0.5 & 0 \\ 0 & 0.5 \end{bmatrix}$ (incerteza do sensor GPS nas duas coordenadas).

Agora o tempo avança para $k = 1$ e o sensor nos dá a nova leitura de posição: $\mathbf{z}_1 = \begin{bmatrix} 1.2 \\ 2.1 \end{bmatrix}$.

### Fase 1: Previsão (Time Update)

#### Passo 1: Previsão do Estado ($\hat{\mathbf{x}}^-$)

Projetamos onde o objeto estará com base na física e na velocidade anterior.

$$
\hat{\mathbf{x}}_1^- = F \cdot \mathbf{x}_0 = \begin{bmatrix} 1 & 0 & 1 & 0 \\ 0 & 1 & 0 & 1 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} 0.0 \\ 0.0 \\ 1.0 \\ 2.0 \end{bmatrix} = \begin{bmatrix} 0.0 + 0 + 1.0 + 0 \\ 0 + 0.0 + 0 + 2.0 \\ 1.0 \\ 2.0 \end{bmatrix} = \begin{bmatrix} 1.0 \\ 2.0 \\ 1.0 \\ 2.0 \end{bmatrix}
$$

* **Significado:** O modelo prevê que o objeto foi de $(0,0)$ para $(1.0, 2.0)$, mantendo as velocidades de $1.0$ e $2.0$.

#### Passo 2: Previsão da Covariância do Erro ($P^-$)

A incerteza cresce devido ao tempo e às limitações do modelo ($Q$).

$$
P_1^- = F \cdot P_0 \cdot F^T + Q = \begin{bmatrix} 2.1 & 0 & 1.0 & 0 \\ 0 & 2.1 & 0 & 1.0 \\ 1.0 & 0 & 1.1 & 0 \\ 0 & 1.0 & 0 & 1.1 \end{bmatrix}
$$

### Fase 2: Atualização / Correção (Measurement Update)

O sensor entrega a leitura $\mathbf{z}_1 = \begin{bmatrix} 1.2 \\ 2.1 \end{bmatrix}$.

#### Passo 3: Cálculo do Ganho de Kalman ($K$)

O Ganho de Kalman em 2D é uma matriz $4 \times 2$ que determina quanto peso dar ao sensor para cada uma das 4 variáveis do estado.

* Fórmula: $K = P^- H^T (H P^- H^T + R)^{-1}$

* Calculando a parte interna (Inovação / Incerteza combinada $S$):
  

  $$
  S = H P_1^- H^T + R = \begin{bmatrix} 2.1 & 0 \\ 0 & 2.1 \end{bmatrix} + \begin{bmatrix} 0.5 & 0 \\ 0 & 0.5 \end{bmatrix} = \begin{bmatrix} 2.6 & 0 \\ 0 & 2.6 \end{bmatrix}
  $$

* Invertendo $S$ e multiplicando pelo restante, obtemos o **Ganho de Kalman (**$K$**)**:
  

  $$
  K_1 \approx \begin{bmatrix} 0.808 & 0 \\ 0 & 0.808 \\ 0.385 & 0 \\ 0 & 0.385 \end{bmatrix}
  $$

* **Significado:** As duas primeiras linhas de $K$ mostram que daremos cerca de $81\%$ de peso para a leitura do sensor nas posições $X$ e $Y$. As linhas 3 e 4 mostram como a velocidade também é corrigida indiretamente pela correlação espacial ($38.5\%$).

#### Passo 4: Atualização do Estado ($\hat{\mathbf{x}}$)

Ajustamos a posição e a velocidade previstas somando o erro do sensor ponderado pelo Ganho de Kalman.

1. **Erro de medição (Inovação):**
   

   $$
   \text{Erro} = \mathbf{z}_1 - H \hat{\mathbf{x}}_1^- = \begin{bmatrix} 1.2 \\ 2.1 \end{bmatrix} - \begin{bmatrix} 1.0 \\ 2.0 \end{bmatrix} = \begin{bmatrix} 0.2 \\ 0.1 \end{bmatrix}
   $$

2. **Correção aplicada:**
   

   $$
   K \cdot (\text{Erro}) \approx \begin{bmatrix} 0.808 \cdot 0.2 \\ 0.808 \cdot 0.1 \\ 0.385 \cdot 0.2 \\ 0.385 \cdot 0.1 \end{bmatrix} = \begin{bmatrix} 0.162 \\ 0.081 \\ 0.077 \\ 0.038 \end{bmatrix}
   $$

3. **Estado Atualizado Final (**$\hat{\mathbf{x}}_1$**):**
   

   $$
   \hat{\mathbf{x}}_1 = \begin{bmatrix} 1.0 \\ 2.0 \\ 1.0 \\ 2.0 \end{bmatrix} + \begin{bmatrix} 0.162 \\ 0.081 \\ 0.077 \\ 0.038 \end{bmatrix} = \begin{bmatrix} \mathbf{1.162} \\ \mathbf{2.081} \\ \mathbf{1.077} \\ \mathbf{2.038} \end{bmatrix}
   $$

* **Significado:** A posição estimada foi ajustada para $(1.16, 2.08)$ — um meio-termo refinado entre a previsão física e o GPS. Note que a velocidade estimada em $X$ e $Y$ também subiu levemente ($1.0 \rightarrow 1.077$) porque o objeto andou um pouco mais rápido do que o esperado para cobrir aquela distância.

#### Passo 5: Atualização da Covariância do Erro ($P$)

Por fim, recalculamos a matriz de incerteza $P_1$ para o próximo ciclo:

$$
P_1 = (I - K \cdot H) \cdot P_1^-
$$

Como medimos a posição diretamente com o sensor, as incertezas referentes às posições $p_x$ e $p_y$ na matriz $P_1$ encolhem drasticamente, preparando o filtro para repetir o ciclo no instante $k = 2$.