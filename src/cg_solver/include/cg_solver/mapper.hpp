#ifndef CG_SOLVER_MAPPER_HPP
#define CG_SOLVER_MAPPER_HPP

#include "cg_solver/utils.hpp"
#include <vector>
#include <string>
#include <map>

/**
 * Tipos de células no labirinto
 */
enum class CellType
{
    UNKNOWN, // Não explorado
    FREE,    // Espaço livre
    BLOCKED, // Parede/bloqueado
    ROBOT,   // Posição do robô
    TARGET   // Alvo
};

/**
 * Mapper: Constrói mapa incremental do labirinto
 *
 * - Usa coordenadas relativas (começa em 0,0)
 * - Atualiza com dados dos 8 sensores
 * - Converte para grid normalizado para pathfinding
 */
class Mapper
{
public:
    Mapper();

    // Atualiza mapa com leituras dos sensores na posição atual
    void update_map(const Position &robot_pos,
                    const std::map<std::string, std::string> &sensors);

    // Consulta tipo de célula
    CellType get_cell(const Position &pos) const;
    bool is_explored(const Position &pos) const;

    // Fronteira de exploração
    std::vector<Position> get_frontier() const;

    // Conversão para grid 2D
    std::vector<std::vector<std::string>> to_grid() const;

    // Utilidades
    void print_map() const;
    int count_explored() const { return map_.size(); }

    // Posições (coordenadas originais)
    Position get_robot_position() const { return robot_pos_; }
    Position get_target_position() const { return target_pos_; }
    bool target_found() const { return target_found_; }

    // Posições ajustadas para o grid (com offset aplicado)
    Position get_robot_position_in_grid() const
    {
        return Position(robot_pos_.row - min_row_, robot_pos_.col - min_col_);
    }

    Position get_target_position_in_grid() const
    {
        return Position(target_pos_.row - min_row_, target_pos_.col - min_col_);
    }

private:
    std::map<Position, CellType> map_; // Mapa: Position -> CellType

    Position robot_pos_;
    Position target_pos_;
    bool target_found_;

    // Bounds para conversão de coordenadas
    int min_row_, max_row_;
    int min_col_, max_col_;

    void update_bounds(const Position &pos);
    CellType char_to_cell_type(const std::string &c) const;
};

#endif
