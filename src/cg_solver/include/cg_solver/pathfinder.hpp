#ifndef CG_SOLVER_PATHFINDER_HPP
#define CG_SOLVER_PATHFINDER_HPP

#include "cg_solver/utils.hpp"
#include <vector>
#include <string>
#include <map>

// Classe que implementa algoritmos de busca de caminho
class Pathfinder
{
public:
    Pathfinder(const std::vector<std::vector<std::string>> &grid);

    // BFS: encontra o caminho mais curto (Parte 1)
    std::vector<Position> find_path_bfs(const Position &start, const Position &goal);

    // DFS: explora em profundidade (Parte 2)
    std::vector<Position> find_path_dfs(const Position &start, const Position &goal);

    bool is_valid_position(const Position &pos) const;
    bool is_walkable(const Position &pos) const;
    void print_grid() const;
    void print_path(const std::vector<Position> &path) const;

private:
    std::vector<std::vector<std::string>> grid_; // O mapa do labirinto
    int rows_;                                   // Número de linhas
    int cols_;                                   // Número de colunas

    // Reconstrói o caminho a partir do mapa de pais
    std::vector<Position> reconstruct_path(
        const std::map<Position, Position> &parent_map,
        const Position &start,
        const Position &goal);

    // Retorna vizinhos válidos de uma posição
    std::vector<Position> get_neighbors(const Position &pos);
};

#endif
