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
    const std::vector<std::pair<std::string, std::pair<int, int>>> directions = {
        {"up", {0, 1}}, {"down", {0, -1}}, {"left", {-1, 0}}, {"right", {1, 0}},
        {"up_left", {-1, 1}}, {"up_right", {1, 1}}, {"down_left", {-1, -1}}, {"down_right", {1, -1}}};

    for (const auto &[name, offset] : directions)
    {
        if (sensors.count(name))
        {
            Position pos(robot_pos.row + offset.first, robot_pos.col + offset.second);
            CellType cell = char_to_cell_type(sensors.at(name));

            if (map_.count(pos) && map_[pos] == CellType::TARGET)
                continue;

            map_[pos] = cell;

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
    return map_.count(pos) ? map_.at(pos) : CellType::UNKNOWN;
}

bool Mapper::is_explored(const Position &pos) const
{
    return map_.count(pos);
}

/**
 * Retorna células de fronteira (não exploradas adjacentes a exploradas)
 */
std::vector<Position> Mapper::get_frontier() const
{
    std::set<Position> frontier_set;

    for (const auto &[pos, type] : map_)
    {
        if (type != CellType::FREE && type != CellType::ROBOT)
            continue;

        for (const auto &dir : DIRECTIONS)
        {
            Position neighbor(pos.row + dir.row, pos.col + dir.col);
            if (!is_explored(neighbor))
                frontier_set.insert(neighbor);
        }
    }

    return std::vector<Position>(frontier_set.begin(), frontier_set.end());
}

/**
 * Converte mapa para grid 2D normalizado
 * Grid começa em (0,0) aplicando offset (min_row, min_col)
 */
std::vector<std::vector<std::string>> Mapper::to_grid() const
{
    int rows = max_row_ - min_row_ + 1;
    int cols = max_col_ - min_col_ + 1;

    std::vector<std::vector<std::string>> grid(rows, std::vector<std::string>(cols, "b"));

    const std::map<CellType, std::string> cell_map = {
        {CellType::FREE, "f"}, {CellType::BLOCKED, "b"}, {CellType::ROBOT, "r"},
        {CellType::TARGET, "t"}, {CellType::UNKNOWN, "?"}};

    for (const auto &[pos, type] : map_)
    {
        int r = pos.row - min_row_;
        int c = pos.col - min_col_;
        grid[r][c] = cell_map.at(type);
    }

    return grid;
}

void Mapper::print_map() const
{
    auto grid = to_grid();
    if (grid.empty() || grid[0].empty())
        return;

    std::cout << "\nMapa: " << grid.size() << "x" << grid[0].size() 
              << " | Explorado: " << count_explored() << std::endl;

    const std::map<char, char> display_map = {{'b', '#'}, {'f', ' '}, {'r', 'R'}, {'t', 'T'}, {'?', '?'}};

    for (const auto &row : grid)
    {
        for (const auto &cell : row)
            std::cout << display_map.at(cell[0]);
        std::cout << std::endl;
    }
}

