---
layout: math
title: Início
---

# Filtro de Kalman

Guia passo a passo do Filtro de Kalman, com exemplo numérico e código Python.

## Teoria

- [Algoritmo padrão (5 passos)]({{ '/filtro_de_kalman.html' | relative_url }})
- [Exemplo em uma dimensão (1D)]({{ '/filtro_de_kalman_1d.html' | relative_url }})
- [Exemplo em duas dimensões (2D)]({{ '/filtro_de_kalman_2d.html' | relative_url }})

## Código

- [Notebook Jupyter (HTML)]({{ '/notebook.html' | relative_url }})
- [Notebook no GitHub](https://github.com/antoniomtramos/kalman_filter/blob/main/nb.ipynb)
- [Script Python](https://github.com/antoniomtramos/kalman_filter/blob/main/script_kalman.py)

## Resumo do ciclo

**Previsão:** $$\hat{x}_k^- = F\hat{x}_{k-1}$$ e $$P_k^- = FP_{k-1}F^T + Q$$

**Atualização:** $$K_k = P_k^-H^T(HP_k^-H^T+R)^{-1}$$, $$\hat{x}_k = \hat{x}_k^- + K_k(z_k - H\hat{x}_k^-)$$ e $$P_k = (I-K_kH)P_k^-$$

[Licença MIT](https://github.com/antoniomtramos/kalman_filter/blob/main/LICENSE)
