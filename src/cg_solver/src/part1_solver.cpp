#include <rclcpp/rclcpp.hpp>
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include <chrono>
#include <thread>
#include <queue>
#include <set>
#include <map>
#include <algorithm>

// Simple Position for Part1
struct Pos {
    int row, col;
    Pos(int r = 0, int c = 0) : row(r), col(c) {}
    bool operator==(const Pos &o) const { return row == o.row && col == o.col; }
    bool operator<(const Pos &o) const { return row != o.row ? row < o.row : col < o.col; }
};

/**
 * Part1Solver: Resolve labirinto com mapa completo
 *
 * Algoritmo:
 * 1. Obtém mapa completo via /get_map
 * 2. Encontra posições do robô e alvo no grid
 * 3. Calcula caminho ótimo com BFS
 * 4. Executa movimentos sequencialmente
 */
class Part1Solver : public rclcpp::Node
{
public:
    Part1Solver() : Node("part1_solver")
    {
        map_client_ = create_client<cg_interfaces::srv::GetMap>("/get_map");
        move_client_ = create_client<cg_interfaces::srv::MoveCmd>("/move_command");

        RCLCPP_INFO(get_logger(), "Part 1 Solver started");

        map_client_->wait_for_service();
        move_client_->wait_for_service();
    }

    void run() { solve(); }

private:
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr map_client_;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;

    void solve()
    {
        // 1. Obter mapa completo
        auto grid = get_complete_map();

        // 2. Encontrar posições
        Pos robot_pos, target_pos;
        find_robot_and_target(grid, robot_pos, target_pos);

        // 3. Calcular caminho ótimo (BFS)
        auto path = find_path_bfs(grid, robot_pos, target_pos);
        RCLCPP_INFO(get_logger(), "Path: %zu steps", path.size());

        // 4. Executar caminho
        execute_path(path);

        RCLCPP_INFO(get_logger(), "✓ Part 1 complete!");
    }

    /**
     * Obtém o grid completo do labirinto via serviço
     */
    std::vector<std::vector<std::string>> get_complete_map()
    {
        auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();
        auto future = map_client_->async_send_request(request);
        
        while (rclcpp::ok() && future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
            rclcpp::spin_some(this->get_node_base_interface());
        
        auto result = future.get();
        int rows = result->occupancy_grid_shape[0];
        int cols = result->occupancy_grid_shape[1];

        std::vector<std::vector<std::string>> grid(rows, std::vector<std::string>(cols));
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                grid[i][j] = result->occupancy_grid_flattened[i * cols + j];

        return grid;
    }

    void find_robot_and_target(const std::vector<std::vector<std::string>> &grid,
                               Pos &robot, Pos &target)
    {
        for (size_t i = 0; i < grid.size(); i++)
            for (size_t j = 0; j < grid[i].size(); j++)
            {
                if (grid[i][j] == "r") robot = Pos(i, j);
                if (grid[i][j] == "t") target = Pos(i, j);
            }
    }

    // BFS pathfinding with CORRECT grid[row][col] indexing
    std::vector<Pos> find_path_bfs(const std::vector<std::vector<std::string>> &grid,
                                   const Pos &start, const Pos &goal)
    {
        const std::vector<Pos> dirs = {{-1,0}, {1,0}, {0,-1}, {0,1}}; // up, down, left, right
        
        std::queue<Pos> queue;
        std::set<Pos> visited;
        std::map<Pos, Pos> parent;
        
        queue.push(start);
        visited.insert(start);
        
        while (!queue.empty())
        {
            Pos cur = queue.front();
            queue.pop();
            
            if (cur == goal)
            {
                std::vector<Pos> path;
                for (Pos p = goal; !(p == start); p = parent[p])
                    path.push_back(p);
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                return path;
            }
            
            for (const auto &dir : dirs)
            {
                Pos next(cur.row + dir.row, cur.col + dir.col);
                if (next.row >= 0 && next.row < (int)grid.size() &&
                    next.col >= 0 && next.col < (int)grid[0].size() &&
                    grid[next.row][next.col] != "b" && !visited.count(next))
                {
                    visited.insert(next);
                    parent[next] = cur;
                    queue.push(next);
                }
            }
        }
        return {};
    }

    /**
     * Executa sequência de movimentos do caminho
     */
    void execute_path(const std::vector<Pos> &path)
    {
        for (size_t i = 1; i < path.size(); i++)
        {
            int dr = path[i].row - path[i-1].row;
            int dc = path[i].col - path[i-1].col;
            
            std::string dir_str;
            if (dr == -1) dir_str = "up";
            else if (dr == 1) dir_str = "down";
            else if (dc == -1) dir_str = "left";
            else if (dc == 1) dir_str = "right";

            auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
            request->direction = dir_str;
            auto future = move_client_->async_send_request(request);
            
            while (rclcpp::ok() && future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
                rclcpp::spin_some(this->get_node_base_interface());
            
            auto result = future.get();

            if (result->success)
                RCLCPP_INFO(get_logger(), "Step %zu: %s", i, dir_str.c_str());
            else
                RCLCPP_WARN(get_logger(), "Move %s blocked!", dir_str.c_str());

            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Part1Solver>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
