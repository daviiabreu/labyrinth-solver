#include "cg_solver/pathfinder.hpp"
#include <iostream>
#include <algorithm>
#include <queue>
#include <stack>
#include <map>
#include <set>

Pathfinder::Pathfinder(const std::vector<std::vector<std::string>> &grid)
    : grid_(grid), rows_(grid.size()), cols_(grid.empty() ? 0 : grid[0].size()) {}

// Verifica se uma posição está dentro dos limites do grid
bool Pathfinder::is_valid_position(const Position &pos) const
{
    return pos.row >= 0 && pos.row < rows_ &&
           pos.col >= 0 && pos.col < cols_;
}

// Verifica se uma célula é transitável (não é bloqueada 'b')
bool Pathfinder::is_walkable(const Position &pos) const
{
    if (!is_valid_position(pos))
        return false;
    std::string cell = grid_[pos.row][pos.col];
    return cell != "b"; // 'b' = bloqueado (preto)
}

// Retorna os vizinhos válidos de uma posição
std::vector<Position> Pathfinder::get_neighbors(const Position &pos)
{
    std::vector<Position> neighbors;

    // Tenta cada uma das 4 direções
    for (const auto &dir : DIRECTIONS)
    {
        Position neighbor(pos.row + dir.row, pos.col + dir.col);
        if (is_walkable(neighbor))
        {
            neighbors.push_back(neighbor);
        }
    }

    return neighbors;
}

// Reconstrói o caminho usando o mapa de pais
std::vector<Position> Pathfinder::reconstruct_path(
    const std::map<Position, Position> &parent_map,
    const Position &start,
    const Position &goal)
{
    std::vector<Position> path;
    Position current = goal;

    // Volta do objetivo até o início seguindo os pais
    while (current != start)
    {
        path.push_back(current);
        current = parent_map.at(current);
    }
    path.push_back(start);

    // Inverte para obter o caminho do início ao objetivo
    std::reverse(path.begin(), path.end());
    return path;
}

// BFS
std::vector<Position> Pathfinder::find_path_bfs(
    const Position &start, const Position &goal)
{
    std::queue<Position> queue;

    std::set<Position> visited;

    std::map<Position, Position> parent_map;

    queue.push(start);
    visited.insert(start);

    while (!queue.empty())
    {
        Position current = queue.front();
        queue.pop();

        // Se chegou no objetivo reconstrói e retorna o caminho
        if (current == goal)
        {
            return reconstruct_path(parent_map, start, goal);
        }

        // Explora cada vizinho
        for (const Position &neighbor : get_neighbors(current))
        {
            // Se já visitou, pula
            if (visited.count(neighbor))
                continue;

            // Marca como visitado e guarda o pai
            visited.insert(neighbor);
            parent_map[neighbor] = current;
            queue.push(neighbor);
        }
    }

    // Não encontrou caminho
    return {};
}

// DFS
std::vector<Position> Pathfinder::find_path_dfs(
    const Position &start, const Position &goal)
{
    std::stack<Position> stack;

    // Conjunto de posições já visitadas
    std::set<Position> visited;

    // Mapa que guarda o pai de cada posição
    std::map<Position, Position> parent_map;

    // Começa pela posição inicial
    stack.push(start);
    visited.insert(start);

    // Enquanto houver posições para explorar
    while (!stack.empty())
    {
        Position current = stack.top();
        stack.pop();

        // Chegou no objetivo? Reconstrói e retorna o caminho
        if (current == goal)
        {
            return reconstruct_path(parent_map, start, goal);
        }

        // Explora cada vizinho
        for (const Position &neighbor : get_neighbors(current))
        {
            // Se já visitou, pula
            if (visited.count(neighbor))
                continue;

            // Marca como visitado e guarda o pai
            visited.insert(neighbor);
            parent_map[neighbor] = current;
            stack.push(neighbor);
        }
    }

    // Não encontrou caminho
    return {};
}

void Pathfinder::print_grid() const
{
    for (int i = 0; i < rows_; i++)
    {
        for (int j = 0; j < cols_; j++)
        {
            std::cout << grid_[i][j];
        }
        std::cout << std::endl;
    }
}

void Pathfinder::print_path(const std::vector<Position> &path) const
{
    std::cout << "Caminho: " << path.size() << " passos" << std::endl;
}
