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
    return pos.row >= 0 && pos.row < rows_ && pos.col >= 0 && pos.col < cols_;
}

bool Pathfinder::is_walkable(const Position &pos) const
{
    return is_valid_position(pos) && grid_[pos.row][pos.col] != "b";
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
    for (Position current = goal; current != start; current = parent_map.at(current))
        path.push_back(current);
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

/**
 * BFS (Breadth-First Search) - Garante caminho mais curto
 */
std::vector<Position> Pathfinder::find_path_bfs(const Position &start, const Position &goal)
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

        if (current == goal)
            return reconstruct_path(parent, start, goal);

        for (const Position &neighbor : get_neighbors(current))
        {
            if (!visited.count(neighbor))
            {
                visited.insert(neighbor);
                parent[neighbor] = current;
                queue.push(neighbor);
            }
        }
    }
    return {};
}

/**
 * DFS (Depth-First Search) - Encontra um caminho (não necessariamente o mais curto)
 */
std::vector<Position> Pathfinder::find_path_dfs(const Position &start, const Position &goal)
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

        if (current == goal)
            return reconstruct_path(parent, start, goal);

        for (const Position &neighbor : get_neighbors(current))
        {
            if (!visited.count(neighbor))
            {
                visited.insert(neighbor);
                parent[neighbor] = current;
                stack.push(neighbor);
            }
        }
    }
    return {};
}

void Pathfinder::print_grid() const
{
    for (const auto &row : grid_)
    {
        for (const auto &cell : row)
            std::cout << cell;
        std::cout << std::endl;
    }
}

void Pathfinder::print_path(const std::vector<Position> &path) const
{
    std::cout << "Path: " << path.size() << " steps" << std::endl;
}
