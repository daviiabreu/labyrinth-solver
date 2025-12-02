#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include "cg_interfaces/msg/robot_sensors.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_solver/mapper.hpp"
#include "cg_solver/pathfinder.hpp"
#include <chrono>
#include <thread>
#include <stack>
#include <set>

/**
 * Part2Mapper: Mapeia o labirinto usando DFS com backtracking
 *
 * Algoritmo:
 * 1. Começa em (0,0) usando coordenadas relativas
 * 2. Explora células não visitadas (DFS)
 * 3. Quando não há mais células para explorar, faz backtracking
 * 4. Para quando encontra o alvo em uma direção cardinal
 * 5. Valida se o mapa é suficiente para encontrar a rota ótima
 */
class Part2Mapper : public rclcpp::Node
{
public:
    Part2Mapper() : Node("part2_mapper"), current_pos_(0, 0)
    {
        // Subscribe to sensor data
        rclcpp::QoS qos = rclcpp::SensorDataQoS();
        sensor_sub_ = create_subscription<cg_interfaces::msg::RobotSensors>(
            "/culling_games/robot_sensors", qos,
            [this](const cg_interfaces::msg::RobotSensors::ConstSharedPtr msg)
            {
                last_sensors_ = msg;
                has_sensor_data_ = true;
            });

        // Create service clients
        move_client_ = create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        map_client_ = create_client<cg_interfaces::srv::GetMap>("/get_map");

        move_client_->wait_for_service();

        // Initialize DFS structures
        visited_cells_.insert({0, 0});
        path_stack_.push(current_pos_);

        RCLCPP_INFO(get_logger(), "Mapper ready! Starting DFS exploration from (0,0)");
    }

    void run() { explore_maze(); }

private:
    // ROS components
    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensor_sub_;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr map_client_;

    // Exploration state
    Mapper mapper_;
    Position current_pos_;
    std::set<std::pair<int, int>> visited_cells_; // Células já visitadas
    std::stack<Position> path_stack_;             // Pilha para backtracking (DFS)

    // Sensor data
    cg_interfaces::msg::RobotSensors::ConstSharedPtr last_sensors_;
    bool has_sensor_data_ = false;

    // Direções cardeais: cima, baixo, esquerda, direita
    const std::vector<std::pair<int, int>> DIRECTIONS = {{0, 1}, {0, -1}, {-1, 0}, {1, 0}};
    const std::vector<std::string> DIRECTION_NAMES = {"up", "down", "left", "right"};

    void explore_maze()
    {
        wait_for_sensors();

        for (int step = 0; step < 10000 && rclcpp::ok(); step++)
        {
            update_sensors();
            if (!explore_one_step())
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        RCLCPP_INFO(get_logger(), "✓ Exploration finished!");
    }

    void wait_for_sensors()
    {
        rclcpp::Rate rate(5);
        while (rclcpp::ok() && !has_sensor_data_)
        {
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }
    }

    void update_sensors()
    {
        has_sensor_data_ = false;
        for (int i = 0; i < 20 && !has_sensor_data_; i++)
        {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    bool explore_one_step()
    {
        update_map();

        if (is_target_adjacent())
        {
            RCLCPP_INFO(get_logger(), "🎯 Target found!");
            validate_map();
            return false;
        }

        std::string direction = find_unexplored_direction();
        if (!direction.empty())
        {
            if (execute_move(direction))
            {
                visited_cells_.insert({current_pos_.row, current_pos_.col});
                path_stack_.push(current_pos_);
            }
            return true;
        }

        if (can_backtrack())
        {
            execute_move(get_backtrack_direction());
            return true;
        }

        validate_map();
        return false;
    }

    void update_map()
    {
        if (!last_sensors_) return;

        mapper_.update_map(current_pos_, {
            {"up", last_sensors_->up}, {"down", last_sensors_->down},
            {"left", last_sensors_->left}, {"right", last_sensors_->right},
            {"up_left", last_sensors_->up_left}, {"up_right", last_sensors_->up_right},
            {"down_left", last_sensors_->down_left}, {"down_right", last_sensors_->down_right}});
    }

    bool is_target_adjacent()
    {
        return last_sensors_ && mapper_.target_found() &&
               (last_sensors_->up == "t" || last_sensors_->down == "t" ||
                last_sensors_->left == "t" || last_sensors_->right == "t");
    }

    std::string find_unexplored_direction()
    {
        for (size_t i = 0; i < 4; i++)
        {
            Position next(current_pos_.row + DIRECTIONS[i].first,
                         current_pos_.col + DIRECTIONS[i].second);

            if (visited_cells_.count({next.row, next.col}))
                continue;

            CellType cell = mapper_.get_cell(next);
            if (cell == CellType::FREE || cell == CellType::UNKNOWN)
                return DIRECTION_NAMES[i];
        }
        return "";
    }

    bool can_backtrack() { return path_stack_.size() > 1; }

    std::string get_backtrack_direction()
    {
        path_stack_.pop();
        Position prev = path_stack_.top();
        return direction_to_string({prev.row - current_pos_.row, prev.col - current_pos_.col});
    }

    bool execute_move(const std::string &direction)
    {
        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = direction;
        auto future = move_client_->async_send_request(request);

        for (int i = 0; i < 200 && rclcpp::ok(); i++)
        {
            rclcpp::spin_some(this->get_node_base_interface());

            if (future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready)
            {
                if (!future.get()->success)
                    return false;

                const std::map<std::string, std::pair<int, int>> offsets = {
                    {"up", {0, 1}}, {"down", {0, -1}}, {"left", {-1, 0}}, {"right", {1, 0}}};

                auto offset = offsets.at(direction);
                current_pos_.row += offset.first;
                current_pos_.col += offset.second;

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                return true;
            }
        }
        return false;
    }

    void validate_map()
    {
        auto mapped_grid = mapper_.to_grid();
        auto real_grid = get_real_map();

        Pathfinder pf_mapped(mapped_grid);
        auto path_mapped = pf_mapped.find_path_bfs(
            mapper_.get_robot_position_in_grid(),
            mapper_.get_target_position_in_grid());

        Position robot_real, target_real;
        find_positions_in_grid(real_grid, robot_real, target_real);
        Pathfinder pf_real(real_grid);
        auto path_real = pf_real.find_path_bfs(robot_real, target_real);

        RCLCPP_INFO(get_logger(), "Mapped: %zu steps | Real: %zu steps",
                    path_mapped.size(), path_real.size());

        if (path_mapped.size() == path_real.size() && !path_mapped.empty())
            RCLCPP_INFO(get_logger(), "✓ SUCCESS: Optimal route found!");
        else
            RCLCPP_WARN(get_logger(), "⚠ Suboptimal path");
    }

    std::vector<std::vector<std::string>> get_real_map()
    {
        auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();
        auto future = map_client_->async_send_request(request);

        for (int i = 0; i < 500 && rclcpp::ok(); i++)
        {
            rclcpp::spin_some(this->get_node_base_interface());

            if (future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready)
            {
                auto result = future.get();
                int rows = result->occupancy_grid_shape[0];
                int cols = result->occupancy_grid_shape[1];

                std::vector<std::vector<std::string>> grid(rows, std::vector<std::string>(cols));
                for (int i = 0; i < rows; i++)
                    for (int j = 0; j < cols; j++)
                        grid[i][j] = result->occupancy_grid_flattened[i * cols + j];
                return grid;
            }
        }
        return {};
    }

    void find_positions_in_grid(const std::vector<std::vector<std::string>> &grid,
                                Position &robot, Position &target)
    {
        for (size_t i = 0; i < grid.size(); i++)
            for (size_t j = 0; j < grid[i].size(); j++)
            {
                if (grid[i][j] == "r") robot = Position(i, j);
                if (grid[i][j] == "t") target = Position(i, j);
            }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Part2Mapper>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
