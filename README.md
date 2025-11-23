# Projeto Culling Games (cg)

Jogo de navegação em labirinto usando ROS 2.

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

**Opções:**

- Labirinto aleatório (padrão): `ros2 run cg maze`

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
