#include "cg_solver/mapper.hpp"
#include <iostream>
#include <algorithm>
#include <set>

/**
 * Mapper: Constrói um mapa incremental do labirinto
 *
 * Usa coordenadas relativas começando em (0,0)
 * Mantém track de min/max para converter para grid normalizado
 */

Mapper::Mapper()
    : robot_pos_(0, 0), target_pos_(0, 0), target_found_(false),
      min_row_(0), max_row_(0), min_col_(0), max_col_(0) {}

CellType Mapper::char_to_cell_type(const std::string &c) const
{
    if (c == "b")
        return CellType::BLOCKED;
    if (c == "f")
        return CellType::FREE;
    if (c == "r")
        return CellType::ROBOT;
    if (c == "t")
        return CellType::TARGET;
    return CellType::UNKNOWN;
}

void Mapper::update_bounds(const Position &pos)
{
    min_row_ = std::min(min_row_, pos.row);
    max_row_ = std::max(max_row_, pos.row);
    min_col_ = std::min(min_col_, pos.col);
    max_col_ = std::max(max_col_, pos.col);
}

/**
 * Atualiza o mapa com leituras dos 8 sensores
 */
void Mapper::update_map(const Position &robot_pos,
                        const std::map<std::string, std::string> &sensors)
{
    robot_pos_ = robot_pos;
    map_[robot_pos] = CellType::ROBOT;
    update_bounds(robot_pos);

    // 8 direções: 4 cardeais + 4 diagonais
    struct SensorDir
    {
        std::string name;
        int dr, dc; // delta row, delta col
    };

    std::vector<SensorDir> directions = {
        {"up", 0, 1},
        {"down", 0, -1},
        {"left", -1, 0},
        {"right", 1, 0},
        {"up_left", -1, 1},
        {"up_right", 1, 1},
        {"down_left", -1, -1},
        {"down_right", 1, -1}};

    // Para cada sensor, atualiza a célula adjacente
    for (const auto &dir : directions)
    {
        if (sensors.count(dir.name))
        {
            Position pos(robot_pos.row + dir.dr, robot_pos.col + dir.dc);
            CellType cell = char_to_cell_type(sensors.at(dir.name));

            // Não sobrescreve alvo já encontrado
            if (!(map_.count(pos) && map_[pos] == CellType::TARGET))
            {
                map_[pos] = cell;
            }

            if (cell == CellType::TARGET)
            {
                target_pos_ = pos;
                target_found_ = true;
            }

            update_bounds(pos);
        }
    }
}

CellType Mapper::get_cell(const Position &pos) const
{
    if (map_.count(pos))
        return map_.at(pos);
    return CellType::UNKNOWN;
}

bool Mapper::is_explored(const Position &pos) const
{
    return map_.count(pos) > 0;
}

/**
 * Retorna células de fronteira (não exploradas adjacentes a exploradas)
 */
std::vector<Position> Mapper::get_frontier() const
{
    std::vector<Position> frontier;
    std::set<Position> checked;

    for (const auto &[pos, type] : map_)
    {
        if (type == CellType::FREE || type == CellType::ROBOT)
        {
            for (const auto &dir : DIRECTIONS)
            {
                Position neighbor(pos.row + dir.row, pos.col + dir.col);

                if (!is_explored(neighbor) && !checked.count(neighbor))
                {
                    frontier.push_back(neighbor);
                    checked.insert(neighbor);
                }
            }
        }
    }

    return frontier;
}

/**
 * Converte mapa para grid 2D normalizado
 * Grid começa em (0,0) aplicando offset (min_row, min_col)
 */
std::vector<std::vector<std::string>> Mapper::to_grid() const
{
    int rows = max_row_ - min_row_ + 1;
    int cols = max_col_ - min_col_ + 1;

    // Inicializa grid com paredes
    std::vector<std::vector<std::string>> grid(rows,
                                               std::vector<std::string>(cols, "b"));

    // Preenche com células exploradas
    for (const auto &[pos, type] : map_)
    {
        int r = pos.row - min_row_;
        int c = pos.col - min_col_;

        switch (type)
        {
        case CellType::FREE:
            grid[r][c] = "f";
            break;
        case CellType::BLOCKED:
            grid[r][c] = "b";
            break;
        case CellType::ROBOT:
            grid[r][c] = "r";
            break;
        case CellType::TARGET:
            grid[r][c] = "t";
            break;
        case CellType::UNKNOWN:
            grid[r][c] = "?";
            break;
        }
    }

    return grid;
}

void Mapper::print_map() const
{
    auto grid = to_grid();

    if (grid.empty() || grid[0].empty())
    {
        std::cout << "\nMapa vazio" << std::endl;
        return;
    }

    std::cout << "\nMapa: " << grid.size() << "x" << grid[0].size() << std::endl;
    std::cout << "Explorado: " << count_explored() << std::endl;
    std::cout << "Fronteira: " << count_unknown() << std::endl;

    for (const auto &row : grid)
    {
        for (const auto &cell : row)
        {
            char display = cell[0];
            if (display == 'b')
                std::cout << "#";
            else if (display == 'f')
                std::cout << " ";
            else if (display == 'r')
                std::cout << "R";
            else if (display == 't')
                std::cout << "T";
            else
                std::cout << "?";
        }
        std::cout << std::endl;
    }
}

int Mapper::count_explored() const
{
    return map_.size();
}

int Mapper::count_unknown() const
{
    return get_frontier().size();
}
