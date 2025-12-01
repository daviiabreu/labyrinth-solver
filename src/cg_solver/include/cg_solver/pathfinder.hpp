#ifndef CG_SOLVER_PATHFINDER_HPP
#define CG_SOLVER_PATHFINDER_HPP

#include "cg_solver/utils.hpp"
#include <vector>
#include <string>
#include <map>

/**
 * Pathfinder: Encontra caminhos em um grid 2D
 *
 * Implementa dois algoritmos:
 * - BFS: Caminho mais curto (usado na validação)
 * - DFS: Exploração em profundidade
 */
class Pathfinder
{
public:
    Pathfinder(const std::vector<std::vector<std::string>> &grid);

    // BFS: Encontra o caminho MAIS CURTO
    std::vector<Position> find_path_bfs(const Position &start, const Position &goal);

    // DFS: Encontra UM caminho (não necessariamente o mais curto)
    std::vector<Position> find_path_dfs(const Position &start, const Position &goal);

    // Utilidades
    bool is_valid_position(const Position &pos) const;
    bool is_walkable(const Position &pos) const;
    void print_grid() const;
    void print_path(const std::vector<Position> &path) const;

private:
    std::vector<std::vector<std::string>> grid_; // Grid do labirinto
    int rows_, cols_;                            // Dimensões

    // Helper: reconstrói caminho do objetivo até o início
    std::vector<Position> reconstruct_path(
        const std::map<Position, Position> &parent_map,
        const Position &start,
        const Position &goal);

    // Helper: retorna vizinhos válidos (não bloqueados)
    std::vector<Position> get_neighbors(const Position &pos);
};

#endif
