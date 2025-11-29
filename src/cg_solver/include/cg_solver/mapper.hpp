#ifndef CG_SOLVER_MAPPER_HPP
#define CG_SOLVER_MAPPER_HPP

#include "cg_solver/utils.hpp"
#include <vector>
#include <string>
#include <map>

// Tipos de células no mapa
enum class CellType
{
    UNKNOWN, // Ainda não explorado
    FREE,    // Espaço livre (branco)
    BLOCKED, // Bloqueado (preto)
    ROBOT,   // Posição do robô (azul)
    TARGET   // Alvo (vermelho)
};

// Classe responsável por mapear o labirinto incrementalmente
class Mapper
{
public:
    Mapper();

    // Atualiza o mapa com dados dos sensores
    void update_map(const Position &robot_pos,
                    const std::map<std::string, std::string> &sensors);

    CellType get_cell(const Position &pos) const;
    bool is_explored(const Position &pos) const;

    // Retorna células na fronteira (não exploradas adjacentes a exploradas)
    std::vector<Position> get_frontier() const;

    // Converte o mapa para formato grid
    std::vector<std::vector<std::string>> to_grid() const;

    void print_map() const;
    int count_explored() const;
    int count_unknown() const;

    Position get_robot_position() const { return robot_pos_; }
    Position get_target_position() const { return target_pos_; }
    bool target_found() const { return target_found_; }

private:
    std::map<Position, CellType> map_; // Mapa simples (Position -> CellType)
    Position robot_pos_;
    Position target_pos_;
    bool target_found_;

    int min_row_, max_row_;
    int min_col_, max_col_;

    void update_bounds(const Position &pos);
    CellType char_to_cell_type(const std::string &c) const;
};

#endif
