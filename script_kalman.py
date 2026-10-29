#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filtro de Kalman linear: predição, atualização, filtragem, suavização e previsão.

Autor original: amtr (02/04/2021). Revisado, comentado e estendido.

=============================================================================
MAPA DO SCRIPT (onde encontrar cada coisa)
=============================================================================
 1. Ideia geral e notação ............................ (este cabeçalho)
 2. Passos elementares (blocos de construção)
      kf_predict   -> "para onde o estado deve ir?"      (predição)
      gauss_pdf    -> "quão provável é esta medida?"     (auxiliar)
      kf_update    -> "corrijo o estado com a medida"    (atualização)
 3. Processamento de uma série inteira
      kalman_filter   -> aplica predição+atualização em loop (só passado)
      kalman_smooth   -> refina o resultado olhando o futuro  (RTS)
      kalman_forecast -> projeta o futuro sem novas medidas   (previsão)
 4. Modelo de nível local (o mais simples para separar sinal de ruído)
      nivel_local_matrizes, ajustar_nivel_local
 5. Demonstração (bloco `if __name__ == "__main__"`)

=============================================================================
IDEIA GERAL
=============================================================================
Queremos estimar uma quantidade que NÃO vemos diretamente (o "estado" x, o
sinal verdadeiro) a partir de medidas ruidosas (y). O Kalman combina dois
tipos de informação, ponderando cada uma pela sua incerteza:
  (a) o que o MODELO diz que deveria acontecer (predição);
  (b) o que a MEDIDA diz que aconteceu (atualização).
Se a medida é muito ruidosa (R grande), confiamos mais no modelo; se o
modelo é pouco confiável (Q grande), confiamos mais na medida.

Modelo de espaço de estados:
    x_k = A x_{k-1} + B u_k + w_k,   w_k ~ N(0, Q)   (equação de estado)
    y_k = H x_k + v_k,               v_k ~ N(0, R)   (equação de medida)

Notação (n = tamanho do estado, m = tamanho da medida, T = nº de instantes):
    x / X : estado (n,1)               -> o que queremos estimar
    P     : covariância do estado (n,n)-> nossa incerteza sobre x
    A     : transição (n,n)            -> como o estado evolui sozinho
    Q     : ruído de processo (n,n)    -> incerteza do modelo (o que A não explica)
    B, u  : entrada de controle        -> força externa conhecida (opcional)
    y / Y : medida (m,1)               -> o que observamos
    H     : medida (m,n)               -> como o estado aparece na medida
    R     : ruído de medida (m,m)      -> imprecisão do "sensor"
    K     : ganho de Kalman (n,m)      -> peso dado à medida na correção
Séries completas têm forma (T, m); estados guardados, (T, n, 1).
Sufixos: "pred" = a priori (antes da medida), "filt" = a posteriori
(depois da medida), "smooth" = usando toda a série.
"""

import numpy as np
from scipy.optimize import minimize


# =============================================================================
# 2. PASSOS ELEMENTARES
# =============================================================================
def kf_predict(X, P, A, Q, B=None, U=None):
    """
    PREDIÇÃO (a priori): projeta o estado e a incerteza um passo à frente,
    sem olhar para nenhuma medida nova.

    Args:
        X: (n,1) estimativa do estado no passo k-1.
        P: (n,n) covariância do estado no passo k-1.
        A: (n,n) matriz de transição de estado.
        Q: (n,n) covariância do ruído de processo.
        B: (n,l) matriz de efeito da entrada de controle (opcional).
        U: (l,1) entrada de controle (opcional).

    Returns:
        X, P: estado e covariância preditos.

    X^-_k = A X_{k-1} + B U_k
    P^-_k = A P_{k-1} A^T + Q     <- a incerteza sempre CRESCE aqui (soma Q)
    """
    X = A @ X                      # move o estado conforme a dinâmica do modelo
    if B is not None and U is not None:
        X = X + B @ U              # soma o efeito de uma entrada externa conhecida
    P = A @ P @ A.T + Q            # propaga a incerteza e adiciona a do processo
    return X, P


def gauss_pdf(Y, M, S):
    """
    Densidade de probabilidade gaussiana multivariada N(Y; M, S).
    Serve para medir "quão plausível" é a medida Y dado o que o modelo
    previa (média M, covariância S). Usada na verossimilhança.

    Args:
        Y: (m,1) observação.
        M: (m,1) média prevista.
        S: (m,m) covariância prevista.

    Returns:
        (pdf, log_pdf), ambos escalares. Prefira o log_pdf: o pdf pode
        ficar tão pequeno que vira 0 no computador.
    """
    d = Y - M                                   # diferença entre medida e previsão
    m = M.shape[0]
    _, logdet = np.linalg.slogdet(S)            # log do determinante (estável)
    # log N(y; M, S) = -1/2 [ m·log(2π) + log|S| + d^T S^-1 d ]
    log_pdf = -0.5 * (m * np.log(2 * np.pi) + logdet + (d.T @ np.linalg.solve(S, d)).item())
    return float(np.exp(log_pdf)), float(log_pdf)


def kf_update(X, P, Y, H, R):
    """
    ATUALIZAÇÃO (a posteriori): corrige a predição usando a medida Y.

    Passos: (1) prevê a medida; (2) calcula a "inovação" (surpresa = medida -
    previsão); (3) calcula o ganho K; (4) corrige estado e incerteza.

    Returns:
        X:  (n,1) estado atualizado.
        P:  (n,n) covariância atualizada (forma de Joseph, simétrica e estável).
        K:  (n,m) ganho de Kalman. K próximo de 0: ignora a medida; K grande:
            segue a medida.
        IM: (m,1) média preditiva de Y (o que esperávamos medir).
        IS: (m,m) covariância da inovação (incerteza total da medida esperada).
        LL: log-verossimilhança da medida (usada para estimar Q e R).
    """
    IM = H @ X                                  # medida que o modelo esperava
    IS = R + H @ P @ H.T                        # incerteza = ruído do sensor + incerteza do estado
    K = np.linalg.solve(IS, H @ P).T            # K = P H^T IS^-1 (IS é simétrica)
    X = X + K @ (Y - IM)                        # corrige: estado += ganho * inovação
    I_KH = np.eye(P.shape[0]) - K @ H
    P = I_KH @ P @ I_KH.T + K @ R @ K.T         # incerteza encolhe (forma de Joseph)
    _, LL = gauss_pdf(Y, IM, IS)                # quão provável era essa medida
    return X, P, K, IM, IS, LL


# =============================================================================
# 3. PROCESSAMENTO DE UMA SÉRIE INTEIRA
# =============================================================================
def kalman_filter(Y, A, H, Q, R, X0, P0, B=None, U=None):
    """
    FILTRAGEM: percorre a série em ordem; a estimativa em k usa só medidas
    até k (é o que se poderia fazer em tempo real, sem "olhar o futuro").
    A cada instante: prediz -> (se houver medida) atualiza.
    Medidas NaN são tratadas como faltantes: só há predição naquele passo
    e a incerteza cresce até a próxima medida válida.

    Args:
        Y: (T,m) série observada.
        X0, P0: estado e covariância iniciais (antes da primeira observação).
               P0 grande = "não sei quase nada", o filtro aprende rápido.

    Returns:
        dict com X_pred, P_pred (a priori), X_filt, P_filt (a posteriori),
        innov (inovações = medida - previsão) e loglik (log-verossimilhança
        total; quanto maior, melhor o modelo explica os dados).
    """
    Y = np.asarray(Y, dtype=float)
    if Y.ndim == 1:
        Y = Y[:, None]
    T, n = Y.shape[0], A.shape[0]
    X, P = X0.copy(), P0.copy()
    # arrays onde guardamos o histórico de cada passo
    X_pred, P_pred = np.empty((T, n, 1)), np.empty((T, n, n))
    X_filt, P_filt = np.empty((T, n, 1)), np.empty((T, n, n))
    innov = np.full(Y.shape, np.nan)
    loglik = 0.0
    for k in range(T):
        X, P = kf_predict(X, P, A, Q, B, U)             # 1) predição
        X_pred[k], P_pred[k] = X, P
        if not np.isnan(Y[k]).any():                    # 2) atualização (se há medida)
            yk = Y[k][:, None]
            X, P, _, IM, _, LL = kf_update(X, P, yk, H, R)
            innov[k] = (yk - IM).ravel()
            loglik += LL
        X_filt[k], P_filt[k] = X, P
    return {"X_pred": X_pred, "P_pred": P_pred, "X_filt": X_filt,
            "P_filt": P_filt, "innov": innov, "loglik": loglik}


def kalman_smooth(res, A):
    """
    SUAVIZAÇÃO de Rauch-Tung-Striebel (RTS): refaz as estimativas usando TODAS
    as observações (passado E futuro). Resulta em uma curva mais lisa e mais
    precisa que a filtragem, mas só serve para análise ex-post (olha o futuro).

    Funcionamento: parte da última estimativa filtrada (que já usa toda a
    série) e caminha de trás para frente, corrigindo cada instante com a
    informação do instante seguinte.

    Args:
        res: dicionário devolvido por kalman_filter.
        A: matriz de transição.

    Returns:
        X_smooth (T,n,1) e P_smooth (T,n,n).
    """
    X_f, P_f, X_p, P_p = res["X_filt"], res["P_filt"], res["X_pred"], res["P_pred"]
    T = X_f.shape[0]
    X_s, P_s = X_f.copy(), P_f.copy()      # o último instante já está suavizado
    for k in range(T - 2, -1, -1):         # de trás para frente
        J = P_f[k] @ A.T @ np.linalg.inv(P_p[k + 1])        # ganho da suavização
        X_s[k] = X_f[k] + J @ (X_s[k + 1] - X_p[k + 1])     # puxa para o "futuro" já suavizado
        P_s[k] = P_f[k] + J @ (P_s[k + 1] - P_p[k + 1]) @ J.T
    return X_s, P_s


def kalman_forecast(X, P, A, H, Q, R, steps, B=None, U=None):
    """
    PREVISÃO de `steps` passos à frente a partir do último estado filtrado:
    só repete a predição (sem medidas novas). A incerteza cresce a cada passo,
    então as bandas de confiança abrem conforme o horizonte aumenta.

    Args:
        X, P: último estado e covariância filtrados (X_filt[-1], P_filt[-1]).
        steps: horizonte de previsão.

    Returns:
        y_mean (steps,m) e y_var (steps,m): média e variância preditivas de y
        (a variância inclui o ruído de medida R).
    """
    ym, yv = [], []
    for _ in range(steps):
        X, P = kf_predict(X, P, A, Q, B, U)
        ym.append((H @ X).ravel())                       # previsão da medida
        yv.append(np.diag(H @ P @ H.T + R))              # incerteza dessa previsão
    return np.array(ym), np.array(yv)


# =============================================================================
# 4. MODELO DE NÍVEL LOCAL (sinal = passeio aleatório, ruído = branco)
# =============================================================================
def nivel_local_matrizes(q, r):
    """
    Matrizes do modelo de nível local (estado de dimensão 1):
        x_k = x_{k-1} + w   -> o sinal muda devagar, de forma aleatória (var q)
        y_k = x_k + v       -> medimos o sinal somado a um ruído branco (var r)
    Ou seja, A = 1, H = 1, Q = [[q]], R = [[r]].
    O sinal é o estado x; o ruído é y - x.
    """
    return np.eye(1), np.eye(1), np.array([[q]]), np.array([[r]])


def ajustar_nivel_local(y):
    """
    Estima as variâncias do ruído de processo (q) e de medida (r) por máxima
    verossimilhança: procura os valores que tornam os dados mais prováveis.
    A razão q/r controla a suavidade do sinal:
        q/r pequeno -> sinal muito suave (quase tudo é considerado ruído);
        q/r grande  -> sinal acompanha a série (quase tudo é considerado sinal).

    Args:
        y: série 1D.

    Returns:
        q, r (na escala original de y).
    """
    y = np.asarray(y, dtype=float)
    # Padroniza a série para a otimização ficar bem condicionada;
    # as variâncias voltam à escala original no final.
    media, escala = y.mean(), y.std() or 1.0
    z = ((y - media) / escala)[:, None]
    X0, P0 = np.array([[z[0, 0]]]), np.array([[1.0]])

    def neg_loglik(theta):
        # otimizamos log(q), log(r) para garantir variâncias sempre positivas
        q, r = np.exp(theta)
        A, H, Q, R = nivel_local_matrizes(q, r)
        return -kalman_filter(z, A, H, Q, R, X0, P0)["loglik"]   # minimizar -loglik

    sol = minimize(neg_loglik, x0=np.log([0.1, 0.5]), method="L-BFGS-B",
                   bounds=[(-15, 5), (-15, 5)])
    q, r = np.exp(sol.x)
    return q * escala**2, r * escala**2


def sazonal_ar_matrizes(qn, qs, phi, qa, r, periodo=365.25):
    """
    Modelo para séries com CICLO + anomalias persistentes (ex.: temperatura).
    Estado = [nível, seno, cosseno, anomalia]:
        nível    : passeio aleatório lento (variância qn)
        seno/cos : ciclo de período `periodo`, uma rotação por passo (variância qs)
        anomalia : AR(1) com persistência phi (variância qa) -> dias frios/quentes
                   que duram alguns dias; é RUÍDO ESTRUTURADO, não sinal
    Medida: y = nível + ciclo + anomalia + v (v ~ N(0, r)).
    O sinal é nível + ciclo (H_sinal = [1, 1, 0, 0]); o ruído é o resto.
    """
    w = 2 * np.pi / periodo
    c, s = np.cos(w), np.sin(w)
    A = np.eye(4)
    A[1:3, 1:3] = [[c, s], [-s, c]]
    A[3, 3] = phi
    Q = np.diag([qn, qs, qs, qa])
    H = np.array([[1.0, 1.0, 0.0, 1.0]])
    H_sinal = np.array([[1.0, 1.0, 0.0, 0.0]])
    return A, H, Q, np.array([[r]]), H_sinal


def ajustar_sazonal_ar(y, periodo=365.25):
    """
    Estima (qn, qs, phi, qa, r) do modelo sazonal+AR(1) por máxima
    verossimilhança. Por que não o nível local nessa série? Com anomalias
    persistentes, o nível local não consegue chamá-las de ruído (ruído branco
    não tem memória) e a máxima verossimilhança zera r: o "sinal" vira a
    própria série (overfitting, R2 = 1). O termo AR(1) dá um lugar para essa
    memória que não é o sinal.

    Returns:
        dict com as variâncias na escala original de y e phi.
    """
    y = np.asarray(y, dtype=float)
    media, escala = y.mean(), y.std() or 1.0
    z = ((y - media) / escala)[:, None]
    X0, P0 = np.zeros((4, 1)), np.eye(4)
    X0[0, 0] = z[0, 0]

    def desempacota(th):
        # exp(.) mantém variâncias > 0; a logística mantém 0 < phi < 1
        return np.exp(th[0]), np.exp(th[1]), 1 / (1 + np.exp(-th[2])), np.exp(th[3]), np.exp(th[4])

    def neg_loglik(th):
        A, H, Q, R, _ = sazonal_ar_matrizes(*desempacota(th), periodo=periodo)
        return -kalman_filter(z, A, H, Q, R, X0, P0)["loglik"]

    th0 = [np.log(1e-5), np.log(1e-5), 1.0, np.log(0.1), np.log(0.01)]
    sol = minimize(neg_loglik, th0, method="L-BFGS-B",
                   bounds=[(-18, 0), (-18, 0), (-4, 6), (-12, 1), (-12, 1)])
    qn, qs, phi, qa, r = desempacota(sol.x)
    e2 = escala**2
    return {"qn": qn * e2, "qs": qs * e2, "phi": phi, "qa": qa * e2, "r": r * e2}


# =============================================================================
# 5. DEMONSTRAÇÃO (só roda com `python script_kalman.py`, não ao importar)
# =============================================================================
if __name__ == "__main__":
    # Exemplo: posição 2D com velocidade constante; estado = [x, y, vx, vy].
    # Só medimos a posição (com ruído); o filtro também infere a velocidade.
    rng = np.random.default_rng(0)
    dt = 0.1
    A = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], float)  # posição += v*dt
    H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], float)     # medimos apenas x e y
    Q, R = 0.01 * np.eye(4), 0.5 * np.eye(2)
    # simula a trajetória verdadeira e as medidas ruidosas
    verdade = np.array([[0.0], [0.0], [1.0], [1.0]])
    obs = []
    for _ in range(100):
        verdade = A @ verdade
        obs.append((H @ verdade).ravel() + rng.normal(0, np.sqrt(0.5), 2))
    obs = np.array(obs)
    # filtra, suaviza e prevê 5 passos
    res = kalman_filter(obs, A, H, Q, R, np.zeros((4, 1)), np.eye(4))
    X_s, _ = kalman_smooth(res, A)
    prev, _ = kalman_forecast(res["X_filt"][-1], res["P_filt"][-1], A, H, Q, R, 5)
    print("Estado filtrado final:", res["X_filt"][-1].ravel())
    print("Estado suavizado final:", X_s[-1].ravel())
    print("Previsão 5 passos:\n", prev)
