#ifndef CG_SOLVER_PATHFINDER_HPP
#define CG_SOLVER_PATHFINDER_HPP

#include "cg_solver/utils.hpp"
#include <vector>
#include <string>

class Pathfinder
{
public:
    Pathfinder(const std::vector<std::vector<std::string>> &grid);

    std::vector<Position> find_path_bfs(const Position &start, const Position &goal);
    std::vector<Position> find_path_dfs(const Position &start, const Position &goal);

    bool is_valid_position(const Position &pos) const;
    bool is_walkable(const Position &pos) const;
    void print_grid() const;
    void print_path(const std::vector<Position> &path) const;

private:
    std::vector<std::vector<std::string>> grid_;
    int rows_;
    int cols_;

    std::vector<Position> reconstruct_path(Node *goal_node);
    std::vector<Position> get_neighbors(const Position &pos);
};

#endif
