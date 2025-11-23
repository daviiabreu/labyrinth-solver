#include "cg_solver/pathfinder.hpp"
#include <iostream>
#include <algorithm>
#include <queue>
#include <stack>
#include <unordered_map>
#include <unordered_set>

Pathfinder::Pathfinder(const std::vector<std::vector<std::string>> &grid)
    : grid_(grid), rows_(grid.size()), cols_(grid[0].size()) {}

bool Pathfinder::is_valid_position(const Position &pos) const
{
    return pos.row >= 0 && pos.row < rows_ &&
           pos.col >= 0 && pos.col < cols_;
}

bool Pathfinder::is_walkable(const Position &pos) const
{
    if (!is_valid_position(pos))
        return false;
    std::string cell = grid_[pos.row][pos.col];
    return cell != "b";
}

std::vector<Position> Pathfinder::get_neighbors(const Position &pos)
{
    std::vector<Position> neighbors;

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

std::vector<Position> Pathfinder::reconstruct_path(Node *goal_node)
{
    std::vector<Position> path;
    Node *current = goal_node;

    while (current != nullptr)
    {
        path.push_back(current->pos);
        current = current->parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Position> Pathfinder::find_path_bfs(
    const Position &start, const Position &goal)
{

    std::queue<Node *> queue;
    std::unordered_set<Position, PositionHash> visited;

    Node *start_node = new Node(start);
    queue.push(start_node);
    visited.insert(start);

    std::vector<Node *> all_nodes;
    all_nodes.push_back(start_node);

    while (!queue.empty())
    {
        Node *current = queue.front();
        queue.pop();

        if (current->pos == goal)
        {
            auto path = reconstruct_path(current);
            for (auto node : all_nodes)
            {
                delete node;
            }
            return path;
        }

        for (const Position &neighbor_pos : get_neighbors(current->pos))
        {
            if (visited.count(neighbor_pos))
                continue;

            visited.insert(neighbor_pos);
            Node *neighbor_node = new Node(neighbor_pos, current);
            all_nodes.push_back(neighbor_node);
            queue.push(neighbor_node);
        }
    }

    for (auto node : all_nodes)
    {
        delete node;
    }

    return {};
}

std::vector<Position> Pathfinder::find_path_dfs(
    const Position &start, const Position &goal)
{

    std::stack<Node *> stack;
    std::unordered_set<Position, PositionHash> visited;

    Node *start_node = new Node(start);
    stack.push(start_node);
    visited.insert(start);

    std::vector<Node *> all_nodes;
    all_nodes.push_back(start_node);

    while (!stack.empty())
    {
        Node *current = stack.top();
        stack.pop();

        if (current->pos == goal)
        {
            auto path = reconstruct_path(current);
            for (auto node : all_nodes)
            {
                delete node;
            }
            return path;
        }

        auto neighbors = get_neighbors(current->pos);
        std::reverse(neighbors.begin(), neighbors.end());

        for (const Position &neighbor_pos : neighbors)
        {
            if (visited.count(neighbor_pos))
                continue;

            visited.insert(neighbor_pos);
            Node *neighbor_node = new Node(neighbor_pos, current);
            all_nodes.push_back(neighbor_node);
            stack.push(neighbor_node);
        }
    }

    for (auto node : all_nodes)
    {
        delete node;
    }

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
