# Simulação otimizada de pontos de contato em uma mesa de bilhar 2D

Trabalho de Programação de Algoritmos Numéricos em Tempo Real.

Implementa e compara dois algoritmos para encontrar o **ponto de contato**
entre a reta do taco e as bolas de uma mesa de bilhar 2D:

- **Algoritmo 1 — força bruta (T1)**: testa a reta contra todas as bolas a
  cada frame e retorna o contato mais próximo do taco.
- **Algoritmo 2 — rastreamento via EDO (T2)**: mantém o contato encontrado
  e o atualiza incrementalmente, frame a frame, integrando uma equação
  diferencial ordinária derivada do próprio vínculo geométrico — sem
  reprocessar todas as bolas —, reancorando por força bruta a cada `N`
  frames (Melhoria 1).

## Integrantes

| Nome | Matrícula |
|------|-----------|
| Thiago Dutra Rodrigues Paixão | 2020.1.00765-11 |
| Guilherme Alves Freire | 2023.1.00512-11 |
| Victor Hugo Boaventura Pinheiro Alves | 2017.1.02795-11 |

## O problema

A mesa é um retângulo `[0, largura] x [0, altura]` com bolas circulares.
O taco define, a cada instante `t`, uma reta

```
X(s, t) = P(t) + s · d(t),   s ≥ 0
```

onde `P(t)` é a ponta do taco e `d(t)` a direção (unitária) da mirada.
Cada bola `i` tem centro `Cⁱ(t)` e raio `rⁱ` (possivelmente em movimento).
O **ponto de contato** é a interseção de `X(s,t)` com a bola mais próxima do
taco, isto é, o menor `s ≥ 0` tal que

```
|P(t) + s d(t) − Cⁱ(t)|² = (rⁱ)²
```

para alguma bola `i`.

## Algoritmo 1 — força bruta (T1)

Para cada bola, a equação acima é uma quadrática em `s`:

```
s² + 2(V·d) s + (V·V − r²) = 0,   V = P − C
```

(`d` é unitário, então o coeficiente de `s²` é 1). Resolvendo por Bhaskara,
ficamos com até duas raízes; a menor raiz não-negativa é onde a reta *entra*
na bola. Repetindo para as `n` bolas e tomando o menor `s` entre elas,
obtemos o contato mais próximo do taco.

**Custo:** `O(n)` interseções reta-círculo por frame — refeito do zero
sempre, mesmo que o taco e as bolas tenham se movido muito pouco entre um
frame e o outro. Ver [`brute_force_contact`](src/BilharSim.jl).

## Algoritmo 2 — rastreamento do ponto de contato via EDO (T2)

A ideia é não jogar fora a informação do frame anterior. Definimos

```
F(s, t) = |P(t) + s d(t) − C(t)|² − r² = 0
```

para a bola já travada (`ball_index`) e diferenciamos implicitamente em
relação a `t`, mantendo `F ≡ 0` ao longo do tempo:

```
V := P + s d − C
2 V · (P′ + s′ d + s d′ − C′) = 0     (r constante)

        −V · (P′ + s d′ − C′)
  s′ =  ------------------------
                V · d
```

Isso dá uma **EDO de primeira ordem para `s(t)`**. A cada frame:

1. **Preditor de Euler**: `s_pred = s + s′ · dt`, com `P′, d′` aproximados
   por diferença finita entre frames e `C′` = velocidade real da bola
   (conhecida exatamente na simulação, não aproximada).
2. **Corretor de Newton** (1 passo) sobre `F(s) = 0`, já com `P, d, C` do
   frame novo, para conter a deriva numérica do integrador explícito:
   `s ← s_pred − F(s_pred)/F′(s_pred)`.
3. Se o corretor não converge (reta quase tangente, `s` sai da faixa
   válida) ou a cada `recompute_every` frames, o rastreador **reancora**
   com uma chamada de força bruta (T1) — é o único jeito barato de detectar
   que a bola *mais próxima* mudou (troca de alvo), algo que a EDO local,
   por acompanhar só a bola já travada, não enxerga sozinha. Essa é a
   **Melhoria 1** do enunciado: ajustar `recompute_every` troca precisão
   por velocidade.

**Custo:** `O(1)` por frame na maioria dos frames (uma predição + uma
correção, ambas sobre uma única bola), pagando `O(n)` só nos frames de
reancoragem. Ver [`step_tracker!`](src/BilharSim.jl).

## Estrutura do projeto

```
src/BilharSim.jl   núcleo: Ball, Table, algoritmo 1, algoritmo 2 (T1/T2)
src/App.jl          visualização interativa (GLMakie)
scripts/run_app.jl  abre a visualização
scripts/benchmark.jl compara T1 x T2 em função do nº de bolas / recompute_every
test/runtests.jl    testes: geometria, rastreamento vs. força bruta, troca de alvo
docs/apresentacao.md roteiro para a apresentação do trabalho
```

## Como rodar

Requer Julia ≥ 1.10 (testado em 1.12).

```bash
# na raiz do projeto
julia --project=. -e "using Pkg; Pkg.instantiate()"   # primeira vez: instala GLMakie etc.

julia --project=. test/runtests.jl                     # roda os testes
julia --project=. scripts/benchmark.jl                  # roda o benchmark T1 x T2
julia --project=. scripts/run_app.jl                     # abre a visualização interativa
```

Na visualização:

- **Arraste o botão esquerdo do mouse** para mirar o taco (a posição do
  clique é `P`, a posição atual do mouse define `d`).
- As bolas verdes se movem sozinhas e quicam nas bordas — isso exercita o
  termo `C′(t)` da EDO mesmo sem mexer o mouse.
- **× vermelho** = contato calculado pelo algoritmo 1 a cada frame.
  **○ azul** = contato rastreado pelo algoritmo 2. Quando coincidem, o
  rastreamento está fiel à força bruta.
- Teclas `[` / `]` diminuem/aumentam `recompute_every` em tempo real, para
  visualizar o efeito da Melhoria 1 na precisão e no custo (barra de status
  mostra o tempo de cada algoritmo em microssegundos e o *speedup*).

## Resultados

O benchmark (`scripts/benchmark.jl`) mede o custo médio por frame de T1 e
T2 para diferentes números de bolas e valores de `recompute_every`. Como
esperado da análise de complexidade:

- T1 cresce linearmente com o número de bolas.
- T2 é essencialmente constante por frame, e o *speedup* sobre T1 cresce
  tanto com o número de bolas quanto com `recompute_every` (menos
  reancoragens por segundo).
- A precisão do rastreamento (erro entre o `s` do T2 e o `s` "verdadeiro"
  do T1) se mantém pequena (< 1e-3) entre reancoragens, mas pode divergir
  temporariamente logo após uma troca de bola mais próxima, até a próxima
  reancoragem — esse é exatamente o compromisso que `recompute_every`
  controla.

Execução de referência (Julia 1.12.7, Windows, `n_frames = 500` por medição):

```
bolas      T1 força bruta   T2 (recompute=10)      T2 (recompute=30)      T2 (recompute=60)
5          160.8 ns         221.4 ns (×0.7)        111.2 ns (×1.4)        136.8 ns (×1.2)
20         316.4 ns         178.6 ns (×1.8)        369.6 ns (×0.9)        336.4 ns (×0.9)
50         777.8 ns         478.4 ns (×1.6)        381.4 ns (×2.0)        330.8 ns (×2.4)
100        2165.8 ns        480.6 ns (×4.5)        415.2 ns (×5.2)        172.2 ns (×12.6)
300        5317.4 ns        1234.2 ns (×4.3)       310.2 ns (×17.1)       479.8 ns (×11.1)
```

Com poucas bolas (5–20) o ganho é ruidoso/inconsistente — o overhead fixo
de cada chamada domina e T1 já é rápido o bastante. A partir de ~50–100
bolas o padrão previsto pela análise assintótica aparece claramente:
speedups de 4× a 17× para T2, maiores quanto maior `recompute_every`
(menos reancoragens por segundo). _Recomenda-se regerar essa tabela na
máquina usada na apresentação rodando `scripts/benchmark.jl` — o resultado
varia com hardware e ruído de benchmark._

## Limitações e possíveis extensões

- Não há colisão bola-bola nem física completa de bilhar (fora do escopo:
  o trabalho é sobre o ponto de contato do taco, não sobre a dinâmica do
  jogo).
- O corretor de Newton faz apenas 1 passo por frame — suficiente com `dt`
  pequeno (60 fps), mas poderia iterar até convergência para `dt` maiores.
- `recompute_every` é fixo por execução na versão de benchmark; a
  visualização já permite ajustá-lo em tempo real. Uma extensão natural é
  torná-lo adaptativo (recalcular sob demanda quando a distância ao alvo
  atual estiver próxima da de outra bola).
