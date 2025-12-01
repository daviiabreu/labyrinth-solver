# Projeto Culling Games (cg)

Jogo de navegação em labirinto usando ROS 2.

## Demo

<video width="800" controls>
  <source src="./video.mov" type="video/quicktime">
</video>

## Solver Automático (cg_solver)

Implementação em C++ de solução automática para navegação em labirintos usando ROS 2.

### Estrutura do Código

**Componentes principais:**

- **`utils.hpp`**: Estruturas de dados (`Position`, `Node`, `PositionHash`) e funções auxiliares
- **`pathfinder.hpp/cpp`**: Classe `Pathfinder` com implementações de BFS e DFS
- **`mapper.hpp/cpp`**: Classe `Mapper` para mapeamento incremental usando sensores locais
- **`part1_solver.cpp`**: Nó ROS 2 que resolve labirinto com mapa completo
- **`part2_mapper.cpp`**: Nó ROS 2 que mapeia e navega usando sensores

### Algoritmos Implementados

**BFS (Breadth-First Search):**

- Usa `std::queue` para exploração em largura
- Garante caminho ótimo (menor número de passos)
- Usado na Parte 1 e na navegação final da Parte 2

**DFS (Depth-First Search):**

- Usa `std::stack` para exploração em profundidade
- Usado na Parte 2 para exploração de fronteiras desconhecidas

### Funcionalidades

**Parte 1:**

- Obtém mapa completo via serviço `/get_map`
- Localiza robô (`r`) e alvo (`t`) no mapa
- Calcula caminho ótimo com BFS
- Executa movimentos via serviço `/move_command`

**Parte 2:**

- Subscrição ao tópico `/culling_games/robot_sensors` para dados locais
- Mapeamento incremental: atualiza mapa com leituras dos 8 sensores (cardeais + diagonais)
- Exploração: usa DFS para encontrar fronteiras desconhecidas
- Validação: compara mapa mapeado com mapa real e calcula eficiência da rota

## Como Executar

### 1. Entrar no ambiente Nix

```bash
nix develop
```

### 2. Construir o workspace

```bash
colcon build
source install/setup.bash
```

### 3. Iniciar o jogo

```bash
ros2 run cg maze
```

### 4. Controlar o robô

Em um novo terminal (lembre-se de executar `nix develop` e `source install/setup.bash`):

```bash
ros2 run cg_teleop teleop_keyboard
```

## Como Jogar

**Movimento:**

- **Cima:** `w`, `k`, ou ↑
- **Baixo:** `s`, `j`, ou ↓
- **Esquerda:** `a`, `h`, ou ←
- **Direita:** `d`, `l`, ou →

**Reiniciar:**

- **Mesmo labirinto:** `r`
- **Novo labirinto:** `n`

## Executar Solver

### Compilar

```bash
nix develop
colcon build --packages-select cg_solver
source install/setup.bash
./fix_rpath.sh
```

### Parte 1 (Mapa Completo)

```bash
./run_part1.sh
```

### Parte 2 (Mapeamento com Sensores)

```bash
./run_part2.sh
```
