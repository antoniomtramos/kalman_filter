# Filtro de Kalman em 2D

Material didático em português sobre o Filtro de Kalman: teoria passo a passo e um exemplo em Python (rastreamento de posição e velocidade em um plano X-Y com modelo de velocidade constante).

## Conteúdo

- [filtro_de_kalman.md](filtro_de_kalman.md): algoritmo padrão (5 passos) e glossário de símbolos.
- [filtro_de_kalman_em_duas_dimens_es_2d.md](filtro_de_kalman_em_duas_dimens_es_2d.md): exemplo numérico em 2D.
- [nb.ipynb](nb.ipynb): notebook Jupyter com o passo a passo e uma simulação com gráficos.
- [script_kalman.py](script_kalman.py): script Python.
- [index.md](index.md): página inicial do GitHub Pages.

## Como executar

```bash
pip install numpy matplotlib jupyter
jupyter notebook nb.ipynb
```

## Gerar a página HTML

```bash
jupyter nbconvert --to html --execute nb.ipynb --output notebook
```

Faça commit do `notebook.html` e ative o GitHub Pages em *Settings → Pages* (branch `main`, pasta `/root`).

## Licença

[MIT](LICENSE)
