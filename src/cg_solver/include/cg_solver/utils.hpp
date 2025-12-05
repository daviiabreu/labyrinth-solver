#ifndef CG_SOLVER_UTILS_HPP
#define CG_SOLVER_UTILS_HPP

#include <vector>
#include <string>
#include <map>
#include <set>

// Estrutura simples para representar uma posição no grid
struct Position
{
    int row; // linha
    int col; // coluna

    Position() : row(0), col(0) {}
    Position(int r, int c) : row(r), col(c) {}

    // Operador < necessário para usar Position como chave em map/set
    bool operator<(const Position &other) const
    {
        if (row != other.row)
            return row < other.row;
        return col < other.col;
    }

    bool operator==(const Position &other) const
    {
        return row == other.row && col == other.col;
    }

    bool operator!=(const Position &other) const
    {
        return !(*this == other);
    }
};

// As 4 direções de movimento: cima, baixo, esquerda, direita
const std::vector<Position> DIRECTIONS = {
    {0, 1},  // cima (up)
    {0, -1}, // baixo (down)
    {-1, 0}, // esquerda (left)
    {1, 0}   // direita (right)
};

// Converte uma direção para string de comando
inline std::string direction_to_string(const Position &dir)
{
    if (dir.col == 1)
        return "up";
    if (dir.col == -1)
        return "down";
    if (dir.row == -1)
        return "left";
    if (dir.row == 1)
        return "right";
    return "";
}

#endif
