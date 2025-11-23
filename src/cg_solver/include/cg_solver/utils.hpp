#ifndef CG_SOLVER_UTILS_HPP
#define CG_SOLVER_UTILS_HPP

#include <vector>
#include <string>
#include <queue>
#include <unordered_set>
#include <cmath>

struct Position
{
    int row;
    int col;

    Position() : row(0), col(0) {}
    Position(int r, int c) : row(r), col(c) {}

    bool operator==(const Position &other) const
    {
        return row == other.row && col == other.col;
    }

    bool operator!=(const Position &other) const
    {
        return !(*this == other);
    }
};

struct PositionHash
{
    std::size_t operator()(const Position &pos) const
    {
        return std::hash<int>()(pos.row) ^ (std::hash<int>()(pos.col) << 1);
    }
};

struct Node
{
    Position pos;
    Node *parent;

    Node(Position p, Node *par = nullptr)
        : pos(p), parent(par) {}
};

const std::vector<Position> DIRECTIONS = {
    {-1, 0},
    {1, 0},
    {0, -1},
    {0, 1}};

inline std::string direction_to_string(const Position &dir)
{
    if (dir.row == -1)
        return "up";
    if (dir.row == 1)
        return "down";
    if (dir.col == -1)
        return "left";
    if (dir.col == 1)
        return "right";
    return "";
}

inline int manhattan_distance(const Position &a, const Position &b)
{
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

#endif
