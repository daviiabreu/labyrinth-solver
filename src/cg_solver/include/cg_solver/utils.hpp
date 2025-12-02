#ifndef CG_SOLVER_UTILS_HPP
#define CG_SOLVER_UTILS_HPP

#include <vector>
#include <string>

/**
 * Position: Representa uma posição no labirinto
 *
 * Coordenadas:
 * - row: linha (eixo X, esquerda-direita)
 * - col: coluna (eixo Y, baixo-cima)
 *
 * Sistema de coordenadas:
 *   col (Y)
 *      ↑
 *      |
 *      +---→ row (X)
 */
struct Position
{
    int row; // Coordenada X (horizontal)
    int col; // Coordenada Y (vertical)

    Position() : row(0), col(0) {}
    Position(int r, int c) : row(r), col(c) {}

    // Operadores para usar Position em map/set
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

/**
 * Direções cardeais de movimento (para Part2 coordinate system)
 *
 * Mapeamento direção → offset:
 * - up:    (0, +1)  → aumenta coluna (sobe)
 * - down:  (0, -1)  → diminui coluna (desce)
 * - left:  (-1, 0)  → diminui linha (esquerda)
 * - right: (+1, 0)  → aumenta linha (direita)
 */
const std::vector<Position> DIRECTIONS = {
    {0, 1},  // up
    {0, -1}, // down
    {-1, 0}, // left
    {1, 0}   // right
};

/**
 * Converte offset de direção para comando string (Part2 coordinate system)
 */
inline std::string direction_to_string(const Position &dir)
{
    if (dir.col == 1) return "up";
    if (dir.col == -1) return "down";
    if (dir.row == -1) return "left";
    if (dir.row == 1) return "right";
    return "";
}

#endif
