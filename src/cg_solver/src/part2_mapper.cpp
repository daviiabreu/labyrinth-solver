#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include "cg_interfaces/msg/robot_sensors.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_solver/mapper.hpp"
#include "cg_solver/pathfinder.hpp"
#include <chrono>
#include <thread>
#include <stack>
#include <set>
#include <map>

class Part2Mapper : public rclcpp::Node
{
public:
    Part2Mapper() : Node("part2_mapper"), current_pos_(1, 1), mapping_complete_(false)
    {
        sensor_sub_ = create_subscription<cg_interfaces::msg::RobotSensors>(
            "/culling_games/robot_sensors", 10,
            [this](const cg_interfaces::msg::RobotSensors::ConstSharedPtr msg)
            {
                sensor_callback(msg);
            });

        move_client_ = create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        get_map_client_ = create_client<cg_interfaces::srv::GetMap>("/get_map");

        RCLCPP_INFO(get_logger(), "Parte 2 iniciada");

        while (!move_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(get_logger(), "Aguardando /move_command");
        }

        timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&Part2Mapper::exploration_loop, this));
    }

private:
    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensor_sub_;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr get_map_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    Mapper mapper_;
    Position current_pos_;
    bool mapping_complete_;
    bool first_sensor_received_ = false;

    std::stack<Position> exploration_stack_;
    std::set<Position> visited_dfs_; // Simplificado: set ao invés de unordered_set

    void sensor_callback(const cg_interfaces::msg::RobotSensors::ConstSharedPtr msg)
    {
        // Mapa simples com dados dos 8 sensores
        std::map<std::string, std::string> sensors = {
            {"up", msg->up},
            {"down", msg->down},
            {"left", msg->left},
            {"right", msg->right},
            {"up_left", msg->up_left},
            {"up_right", msg->up_right},
            {"down_left", msg->down_left},
            {"down_right", msg->down_right}};

        mapper_.update_map(current_pos_, sensors);
        first_sensor_received_ = true;

        if (mapper_.target_found() && !mapping_complete_)
        {
            RCLCPP_INFO(get_logger(), "Alvo encontrado: (%d, %d)",
                        mapper_.get_target_position().row,
                        mapper_.get_target_position().col);
            mapping_complete_ = true;
            timer_->cancel();

            mapper_.print_map();

            validate_map();
        }
    }

    void exploration_loop()
    {
        if (!first_sensor_received_ || mapping_complete_)
            return;

        if (exploration_stack_.empty())
        {
            auto frontier = mapper_.get_frontier();

            if (frontier.empty())
            {
                RCLCPP_INFO(get_logger(), "Fronteira vazia, mapeamento completo");
                mapping_complete_ = true;
                timer_->cancel();
                validate_map();
                return;
            }

            for (const auto &pos : frontier)
            {
                if (!visited_dfs_.count(pos))
                {
                    exploration_stack_.push(pos);
                }
            }
        }

        if (!exploration_stack_.empty())
        {
            Position target = exploration_stack_.top();
            exploration_stack_.pop();

            auto grid = mapper_.to_grid();
            Position robot_in_grid = world_to_grid(current_pos_);
            Position target_in_grid = world_to_grid(target);

            Pathfinder pathfinder(grid);
            auto path = pathfinder.find_path_dfs(robot_in_grid, target_in_grid);

            if (!path.empty() && path.size() >= 2)
            {
                Position next_in_grid = path[1];
                Position next_world = grid_to_world(next_in_grid);

                Position direction(next_world.row - current_pos_.row,
                                   next_world.col - current_pos_.col);

                execute_move(direction_to_string(direction));
                visited_dfs_.insert(target);
            }
            else
            {
                move_random_free();
            }
        }
        else
        {
            move_random_free();
        }
    }

    Position world_to_grid(const Position &world_pos)
    {
        return world_pos;
    }

    Position grid_to_world(const Position &grid_pos)
    {
        return grid_pos;
    }

    void move_random_free()
    {
        std::vector<std::string> free_dirs;

        auto check_and_add = [&](const Position &dir, const std::string &name)
        {
            Position next(current_pos_.row + dir.row, current_pos_.col + dir.col);
            CellType cell = mapper_.get_cell(next);
            if (cell == CellType::FREE || cell == CellType::UNKNOWN)
            {
                free_dirs.push_back(name);
            }
        };

        check_and_add({-1, 0}, "up");
        check_and_add({1, 0}, "down");
        check_and_add({0, -1}, "left");
        check_and_add({0, 1}, "right");

        if (!free_dirs.empty())
        {
            execute_move(free_dirs[0]);
        }
    }

    void execute_move(const std::string &direction)
    {
        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = direction;

        auto future = move_client_->async_send_request(request);

        auto start = std::chrono::steady_clock::now();
        while (future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            if (std::chrono::steady_clock::now() - start > std::chrono::seconds(5))
            {
                return;
            }
        }

        if (future.valid())
        {
            auto response = future.get();

            if (response->success)
            {
                current_pos_ = Position(response->robot_pos[0], response->robot_pos[1]);
                RCLCPP_INFO(get_logger(), "%s -> (%d, %d)",
                            direction.c_str(), current_pos_.row, current_pos_.col);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(600));
    }

    void validate_map()
    {
        RCLCPP_INFO(get_logger(), "Validando mapa");
        RCLCPP_INFO(get_logger(), "Obtendo mapa real");
        auto real_grid = get_real_map();

        if (real_grid.empty())
        {
            RCLCPP_ERROR(get_logger(), "Erro ao obter mapa real");
            return;
        }

        auto mapped_grid = mapper_.to_grid();

        Position robot_pos = mapper_.get_robot_position();
        Position target_pos = mapper_.get_target_position();

        RCLCPP_INFO(get_logger(), "Robo: (%d, %d)", robot_pos.row, robot_pos.col);
        RCLCPP_INFO(get_logger(), "Alvo: (%d, %d)", target_pos.row, target_pos.col);
        RCLCPP_INFO(get_logger(), "Calculando rota (mapa mapeado, BFS)");
        Pathfinder pathfinder_mapped(mapped_grid);

        Position robot_in_grid = world_to_grid(robot_pos);
        Position target_in_grid = world_to_grid(target_pos);

        auto path_mapped = pathfinder_mapped.find_path_bfs(
            robot_in_grid, target_in_grid);

        RCLCPP_INFO(get_logger(), "Calculando rota (mapa real, BFS)");
        Pathfinder pathfinder_real(real_grid);

        Position robot_real, target_real;
        find_positions_in_grid(real_grid, robot_real, target_real);

        auto path_real = pathfinder_real.find_path_bfs(robot_real, target_real);

        RCLCPP_INFO(get_logger(), "Resultados:");
        RCLCPP_INFO(get_logger(), "Mapa mapeado: %zu passos", path_mapped.size());
        RCLCPP_INFO(get_logger(), "Mapa real: %zu passos", path_real.size());

        if (path_mapped.size() == path_real.size())
        {
            RCLCPP_INFO(get_logger(), "Rota otima confirmada");
        }
        else
        {
            double efficiency = (double)path_real.size() / path_mapped.size() * 100.0;
            RCLCPP_INFO(get_logger(), "Rota sub-otima: %.1f%% eficiencia", efficiency);
        }

        RCLCPP_INFO(get_logger(), "Celulas exploradas: %d", mapper_.count_explored());
    }

    std::vector<std::vector<std::string>> get_real_map()
    {
        auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();
        auto future = get_map_client_->async_send_request(request);

        auto start = std::chrono::steady_clock::now();
        while (future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            if (std::chrono::steady_clock::now() - start > std::chrono::seconds(5))
            {
                return {};
            }
        }

        if (!future.valid())
        {
            return {};
        }

        auto response = future.get();
        int rows = response->occupancy_grid_shape[0];
        int cols = response->occupancy_grid_shape[1];

        std::vector<std::vector<std::string>> grid(rows,
                                                   std::vector<std::string>(cols));

        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                grid[i][j] = response->occupancy_grid_flattened[i * cols + j];
            }
        }

        return grid;
    }

    bool find_positions_in_grid(const std::vector<std::vector<std::string>> &grid,
                                Position &robot, Position &target)
    {
        bool found_robot = false, found_target = false;

        for (size_t i = 0; i < grid.size(); i++)
        {
            for (size_t j = 0; j < grid[i].size(); j++)
            {
                if (grid[i][j] == "r")
                {
                    robot = Position(i, j);
                    found_robot = true;
                }
                if (grid[i][j] == "t")
                {
                    target = Position(i, j);
                    found_target = true;
                }
            }
        }

        return found_robot && found_target;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Part2Mapper>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
