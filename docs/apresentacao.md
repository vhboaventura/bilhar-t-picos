# Roteiro de apresentação

Sugestão de estrutura para a apresentação do trabalho (ajustar aos minutos
disponíveis e dividir entre os integrantes do grupo).

1. **Problema** — o que é o "ponto de contato" numa mesa de bilhar 2D:
   reta do taco `P(t) + s d(t)`, bolas `Cⁱ(t), rⁱ`, queremos o `s` mínimo
   entre todas as bolas. Mostrar o desenho do quadro / diagrama do README.

2. **Algoritmo 1 (força bruta, T1)** — quadrática reta-círculo por bola,
   menor raiz (entrada na bola) vence. Complexidade `O(n)` por frame.
   Na demo, desligar o T2 mentalmente e olhar só o × vermelho.

3. **Por que força bruta não basta em tempo real** — a cada frame, refazer
   `O(n)` interseções quando na prática o taco e as bolas mudam pouco de um
   frame para o outro é desperdício. Motivação para o algoritmo 2.

4. **Algoritmo 2 (T2) = preditor + corretor de Newton** — a raiz `s` do
   frame anterior é um bom chute para o frame novo:
   - **Preditor de ordem 0**: `s_pred = s` (sem informação de movimento).
   - **Preditor de 1ª ordem (Euler)**: `s_pred = s + s′·dt`, com `s′` da EDO
     obtida por diferenciação implícita de `F(s,t) = 0` (fórmula no README).
   - **Corretor de Newton** até convergir (convergência quadrática).
   Mostrar na demo o menu de preditor e o slider de iterações de Newton.

5. **Preditor de ordem 0 vs. Euler** — comparar com o slider de Newton em 0
   (só preditor): a deriva do ordem 0 é enorme, a do Euler é pequena
   (≈4,6 vs. ≈0,065 no teste). Com Newton, ambos chegam à raiz exata, mas o
   Euler gasta menos iterações (~2,3 vs. ~3,3 por frame). Com 1 iteração só o
   Euler converge sem precisar recalcular tudo.

6. **Melhoria 1 — reancoragem por força bruta** — o rastreio local não
   percebe troca de alvo; demonstrar ao vivo com o slider "Reancorar a cada
   N frames" (0 = nunca) e observar o anel vermelho na bola correta e o
   gráfico de erro (ponto vermelho = T2 na bola errada).

7. **Benchmark** — mostrar a tabela do README (`scripts/benchmark.jl`):
   custo por frame de T1 vs. T2 em função do número de bolas; T1 cresce
   linear, T2 é ~constante. Na demo, subir o número de bolas para ~120 e
   mostrar o speedup ao vivo.

8. **Demo ao vivo** — `julia --project=. scripts/run_server.jl` (abre o navegador em http://localhost:8080).

9. **Limitações e extensões** — ver seção correspondente no README.

## Coisas para preparar antes da apresentação

- [ ] Rodar `scripts/benchmark.jl` na máquina da apresentação e comparar
      com a tabela do README.
- [ ] Subir o servidor uma vez antes (a primeira execução compila e leva
      alguns segundos) e abrir a página na tela/projetor que será usada.
- [ ] Definir quem apresenta cada seção.
