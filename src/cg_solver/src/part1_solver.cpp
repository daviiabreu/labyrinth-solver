#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_solver/pathfinder.hpp"
#include <chrono>
#include <thread>

class Part1Solver : public rclcpp::Node
{
public:
    Part1Solver() : Node("part1_solver")
    {
        get_map_client_ = create_client<cg_interfaces::srv::GetMap>("/get_map");
        move_client_ = create_client<cg_interfaces::srv::MoveCmd>("/move_command");

        RCLCPP_INFO(get_logger(), "Parte 1 iniciada");

        while (!get_map_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(get_logger(), "Aguardando /get_map");
        }
        while (!move_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(get_logger(), "Aguardando /move_command");
        }
    }

    void run()
    {
        solve();
    }

private:
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr get_map_client_;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;

    void solve()
    {
        RCLCPP_INFO(get_logger(), "Obtendo mapa");
        auto grid = get_full_map();

        if (grid.empty())
        {
            RCLCPP_ERROR(get_logger(), "Erro ao obter mapa");
            return;
        }

        Position robot_pos, target_pos;
        if (!find_positions(grid, robot_pos, target_pos))
        {
            RCLCPP_ERROR(get_logger(), "Robo ou alvo nao encontrado");
            return;
        }

        RCLCPP_INFO(get_logger(), "Robo: (%d, %d)", robot_pos.row, robot_pos.col);
        RCLCPP_INFO(get_logger(), "Alvo: (%d, %d)", target_pos.row, target_pos.col);
        RCLCPP_INFO(get_logger(), "Calculando caminho (BFS)");
        Pathfinder pathfinder(grid);
        auto path = pathfinder.find_path_bfs(robot_pos, target_pos);

        if (path.empty())
        {
            RCLCPP_ERROR(get_logger(), "Caminho nao encontrado");
            return;
        }

        pathfinder.print_path(path);

        RCLCPP_INFO(get_logger(), "Executando caminho");
        execute_path(path);
        RCLCPP_INFO(get_logger(), "Concluido");
    }

    std::vector<std::vector<std::string>> get_full_map()
    {
        auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();
        auto future = get_map_client_->async_send_request(request);

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(shared_from_this());
        if (executor.spin_until_future_complete(future, std::chrono::seconds(5)) !=
            rclcpp::FutureReturnCode::SUCCESS)
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

    bool find_positions(const std::vector<std::vector<std::string>> &grid,
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

    void execute_path(const std::vector<Position> &path)
    {
        for (size_t i = 1; i < path.size(); i++)
        {
            Position current = path[i - 1];
            Position next = path[i];

            Position direction(next.row - current.row, next.col - current.col);
            std::string dir_str = direction_to_string(direction);

            auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
            request->direction = dir_str;

            auto future = move_client_->async_send_request(request);

            rclcpp::executors::SingleThreadedExecutor executor;
            executor.add_node(shared_from_this());
            if (executor.spin_until_future_complete(future, std::chrono::seconds(5)) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = future.get();

                if (response->success)
                {
                    RCLCPP_INFO(get_logger(), "Passo %zu: %s -> (%d, %d)",
                                i, dir_str.c_str(),
                                response->robot_pos[0], response->robot_pos[1]);
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "Movimento %s bloqueado", dir_str.c_str());
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
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
