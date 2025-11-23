#ifndef CG_SOLVER_MAPPER_HPP
#define CG_SOLVER_MAPPER_HPP

#include "cg_solver/utils.hpp"
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>

enum class CellType
{
    UNKNOWN,
    FREE,
    BLOCKED,
    ROBOT,
    TARGET
};

class Mapper
{
public:
    Mapper();

    void update_map(const Position &robot_pos,
                    const std::unordered_map<std::string, std::string> &sensors);

    CellType get_cell(const Position &pos) const;

    bool is_explored(const Position &pos) const;

    std::vector<Position> get_frontier() const;

    std::vector<std::vector<std::string>> to_grid() const;

    void print_map() const;
    int count_explored() const;
    int count_unknown() const;

    Position get_robot_position() const { return robot_pos_; }
    Position get_target_position() const { return target_pos_; }
    bool target_found() const { return target_found_; }

private:
    std::unordered_map<Position, CellType, PositionHash> map_;
    Position robot_pos_;
    Position target_pos_;
    bool target_found_;

    int min_row_, max_row_;
    int min_col_, max_col_;

    void update_bounds(const Position &pos);
    CellType char_to_cell_type(const std::string &c) const;
};

#endif
