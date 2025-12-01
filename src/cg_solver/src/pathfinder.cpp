#include "cg_solver/pathfinder.hpp"
#include <iostream>
#include <algorithm>
#include <queue>
#include <stack>
#include <map>
#include <set>

/**
 * Pathfinder: Algoritmos de busca em grid
 *
 * BFS garante caminho mais curto
 * DFS é mais simples mas não garante ótimo
 */

Pathfinder::Pathfinder(const std::vector<std::vector<std::string>> &grid)
    : grid_(grid), rows_(grid.size()), cols_(grid.empty() ? 0 : grid[0].size()) {}

bool Pathfinder::is_valid_position(const Position &pos) const
{
    return pos.row >= 0 && pos.row < rows_ &&
           pos.col >= 0 && pos.col < cols_;
}

bool Pathfinder::is_walkable(const Position &pos) const
{
    if (!is_valid_position(pos))
        return false;
    return grid_[pos.row][pos.col] != "b"; // "b" = blocked (parede)
}

std::vector<Position> Pathfinder::get_neighbors(const Position &pos)
{
    std::vector<Position> neighbors;

    for (const auto &dir : DIRECTIONS)
    {
        Position neighbor(pos.row + dir.row, pos.col + dir.col);
        if (is_walkable(neighbor))
            neighbors.push_back(neighbor);
    }

    return neighbors;
}

std::vector<Position> Pathfinder::reconstruct_path(
    const std::map<Position, Position> &parent_map,
    const Position &start,
    const Position &goal)
{
    std::vector<Position> path;
    Position current = goal;

    // Segue os pais do goal até o start
    while (current != start)
    {
        path.push_back(current);
        current = parent_map.at(current);
    }
    path.push_back(start);

    // Inverte: start → goal
    std::reverse(path.begin(), path.end());
    return path;
}

/**
 * BFS (Breadth-First Search)
 *
 * Garante CAMINHO MAIS CURTO
 * Usa fila: explora por níveis
 *
 * Complexidade: O(V + E)
 */
std::vector<Position> Pathfinder::find_path_bfs(
    const Position &start, const Position &goal)
{
    std::queue<Position> queue;
    std::set<Position> visited;
    std::map<Position, Position> parent;

    queue.push(start);
    visited.insert(start);

    while (!queue.empty())
    {
        Position current = queue.front();
        queue.pop();

        // Chegou no objetivo
        if (current == goal)
            return reconstruct_path(parent, start, goal);

        // Explora vizinhos
        for (const Position &neighbor : get_neighbors(current))
        {
            if (visited.count(neighbor))
                continue; // Já visitado

            visited.insert(neighbor);
            parent[neighbor] = current;
            queue.push(neighbor);
        }
    }

    return {}; // Não achou caminho
}

/**
 * DFS (Depth-First Search)
 *
 * Encontra UM caminho (não necessariamente o mais curto)
 * Usa pilha: explora em profundidade
 *
 * Complexidade: O(V + E)
 */
std::vector<Position> Pathfinder::find_path_dfs(
    const Position &start, const Position &goal)
{
    std::stack<Position> stack;
    std::set<Position> visited;
    std::map<Position, Position> parent;

    stack.push(start);
    visited.insert(start);

    while (!stack.empty())
    {
        Position current = stack.top();
        stack.pop();

        // Chegou no objetivo
        if (current == goal)
            return reconstruct_path(parent, start, goal);

        // Explora vizinhos
        for (const Position &neighbor : get_neighbors(current))
        {
            if (visited.count(neighbor))
                continue; // Já visitado

            visited.insert(neighbor);
            parent[neighbor] = current;
            stack.push(neighbor);
        }
    }

    return {}; // Não achou caminho
}

void Pathfinder::print_grid() const
{
    for (int i = 0; i < rows_; i++)
    {
        for (int j = 0; j < cols_; j++)
            std::cout << grid_[i][j];
        std::cout << std::endl;
    }
}

void Pathfinder::print_path(const std::vector<Position> &path) const
{
    std::cout << "Path: " << path.size() << " steps" << std::endl;
}
