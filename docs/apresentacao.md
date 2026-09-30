# Roteiro de apresentação

Sugestão de estrutura para a apresentação do trabalho (ajustar aos minutos
disponíveis e dividir entre os integrantes do grupo).

1. **Problema** — o que é o "ponto de contato" numa mesa de bilhar 2D:
   reta do taco `P(t) + s d(t)`, bolas `Cⁱ(t), rⁱ`, queremos o `s` mínimo
   entre todas as bolas. Mostrar o desenho do quadro / diagrama do README.

2. **Algoritmo 1 (força bruta, T1)** — quadrática reta-círculo por bola,
   menor `s` não-negativo vence. Complexidade `O(n)` por frame. Rodar a
   visualização mostrando só o × vermelho.

3. **Por que força bruta não basta em tempo real** — a cada frame, refazer
   `O(n)` interseções quando na prática o taco e as bolas mudam pouco de um
   frame para o outro é desperdício. Motivação para o algoritmo 2.

4. **Algoritmo 2 (rastreamento via EDO, T2)** — derivação da EDO por
   diferenciação implícita de `F(s,t)=0` (mostrar a fórmula do README),
   preditor de Euler + corretor de Newton. Rodar a visualização mostrando
   o ○ azul sobrepondo o × vermelho.

5. **Melhoria 1 — reancoragem periódica** — por que o rastreador local não
   detecta troca de bola mais próxima sozinho; demonstrar ao vivo trocando
   `recompute_every` com `[`/`]` e observando o atraso na troca de alvo.

6. **Benchmark** — mostrar a tabela/gráfico de `scripts/benchmark.jl`:
   custo por frame de T1 vs. T2 em função do número de bolas, e o
   *speedup*. Destacar que T1 cresce linear e T2 é ~constante.

7. **Demo ao vivo** — `julia --project=. scripts/run_app.jl`.

8. **Limitações e extensões** — ver seção correspondente no README.

## Coisas para preparar antes da apresentação

- [ ] Rodar `scripts/benchmark.jl` na máquina da apresentação e colar a
      tabela de resultados no README.
- [ ] Testar a demo ao vivo com o projetor/tela (`GLMakie` abre uma janela
      nativa — testar resolução antes).
- [ ] Preencher a seção "Integrantes" do README.
- [ ] Definir quem apresenta cada seção.
