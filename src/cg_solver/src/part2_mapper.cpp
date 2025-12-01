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

        // Wait for services
        while (!move_client_->wait_for_service(std::chrono::seconds(1)))
        {
        }

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

    /**
     * Loop principal de exploração DFS
     */
    void explore_maze()
    {
        wait_for_sensors();

        RCLCPP_INFO(get_logger(), "Starting DFS exploration!");

        // Loop de exploração
        for (int step = 0; step < 10000 && rclcpp::ok(); step++)
        {
            update_sensors();

            if (!explore_one_step())
                break; // Exploração completa

            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        RCLCPP_INFO(get_logger(), "Exploration finished!");
    }

    /**
     * Aguarda primeira leitura dos sensores
     */
    void wait_for_sensors()
    {
        rclcpp::Rate rate(5);
        while (rclcpp::ok() && !has_sensor_data_)
        {
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }
    }

    /**
     * Atualiza leitura dos sensores
     */
    void update_sensors()
    {
        has_sensor_data_ = false;
        for (int i = 0; i < 20 && !has_sensor_data_; i++)
        {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    /**
     * Executa um passo de exploração DFS
     * Retorna false se exploração terminou
     */
    bool explore_one_step()
    {
        // Atualiza mapa com sensores atuais
        update_map();

        // Verifica se alvo está adjacente (em direção cardinal)
        if (is_target_adjacent())
        {
            RCLCPP_INFO(get_logger(), "🎯 Target found and reachable!");
            validate_map();
            return false;
        }

        // Tenta explorar nova célula (DFS)
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

        // Backtracking: volta para célula anterior
        if (can_backtrack())
        {
            std::string back_dir = get_backtrack_direction();
            execute_move(back_dir);
            return true;
        }

        // Exploração completa
        RCLCPP_INFO(get_logger(), "✓ Maze fully explored");
        validate_map();
        return false;
    }

    /**
     * Atualiza o mapa com dados dos sensores
     */
    void update_map()
    {
        if (!last_sensors_)
            return;

        std::map<std::string, std::string> sensors = {
            {"up", last_sensors_->up},
            {"down", last_sensors_->down},
            {"left", last_sensors_->left},
            {"right", last_sensors_->right},
            {"up_left", last_sensors_->up_left},
            {"up_right", last_sensors_->up_right},
            {"down_left", last_sensors_->down_left},
            {"down_right", last_sensors_->down_right}};

        mapper_.update_map(current_pos_, sensors);
    }

    /**
     * Verifica se o alvo está em uma direção cardinal (alcançável)
     */
    bool is_target_adjacent()
    {
        if (!mapper_.target_found() || !last_sensors_)
            return false;

        return (last_sensors_->up == "t" ||
                last_sensors_->down == "t" ||
                last_sensors_->left == "t" ||
                last_sensors_->right == "t");
    }

    /**
     * Encontra uma direção não visitada para explorar
     */
    std::string find_unexplored_direction()
    {
        for (size_t i = 0; i < 4; i++)
        {
            Position next(
                current_pos_.row + DIRECTIONS[i].first,
                current_pos_.col + DIRECTIONS[i].second);

            // Célula não visitada e livre/desconhecida
            if (!visited_cells_.count({next.row, next.col}))
            {
                CellType cell = mapper_.get_cell(next);
                if (cell == CellType::FREE || cell == CellType::UNKNOWN)
                {
                    return DIRECTION_NAMES[i];
                }
            }
        }
        return ""; // Nenhuma célula para explorar
    }

    /**
     * Verifica se pode fazer backtracking
     */
    bool can_backtrack()
    {
        return path_stack_.size() > 1;
    }

    /**
     * Calcula direção para backtracking
     */
    std::string get_backtrack_direction()
    {
        path_stack_.pop();
        Position previous = path_stack_.top();

        int dr = previous.row - current_pos_.row;
        int dc = previous.col - current_pos_.col;

        if (dc == 1)
            return "up";
        if (dc == -1)
            return "down";
        if (dr == -1)
            return "left";
        if (dr == 1)
            return "right";

        return "";
    }

    /**
     * Executa um movimento do robô
     * Usa coordenadas RELATIVAS (não absolutas)
     */
    bool execute_move(const std::string &direction)
    {
        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = direction;
        auto future = move_client_->async_send_request(request);

        // Espera resposta processando callbacks
        auto start = std::chrono::steady_clock::now();
        while (rclcpp::ok())
        {
            rclcpp::spin_some(this->get_node_base_interface());

            if (future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready)
            {
                auto response = future.get();
                if (response->success)
                {
                    // Atualiza posição RELATIVAMENTE
                    if (direction == "up")
                        current_pos_.col += 1;
                    else if (direction == "down")
                        current_pos_.col -= 1;
                    else if (direction == "left")
                        current_pos_.row -= 1;
                    else if (direction == "right")
                        current_pos_.row += 1;

                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    return true;
                }
                return false;
            }

            // Timeout após 2 segundos
            if (std::chrono::steady_clock::now() - start > std::chrono::seconds(2))
                return false;

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
    }

    /**
     * Valida se o mapa mapeado é suficiente para encontrar a rota ótima
     */
    void validate_map()
    {
        RCLCPP_INFO(get_logger(), "\n=== VALIDATING MAPPED MAZE ===");

        auto mapped_grid = mapper_.to_grid();
        auto real_grid = get_real_map();

        // Posições ajustadas para o grid (com offset)
        Position robot_mapped = mapper_.get_robot_position_in_grid();
        Position target_mapped = mapper_.get_target_position_in_grid();

        // Encontra caminho no mapa mapeado
        Pathfinder pf_mapped(mapped_grid);
        auto path_mapped = pf_mapped.find_path_bfs(robot_mapped, target_mapped);

        // Encontra caminho no mapa real
        Pathfinder pf_real(real_grid);
        Position robot_real, target_real;
        find_positions_in_grid(real_grid, robot_real, target_real);
        auto path_real = pf_real.find_path_bfs(robot_real, target_real);

        // Compara resultados
        RCLCPP_INFO(get_logger(), "Mapped path: %zu steps", path_mapped.size());
        RCLCPP_INFO(get_logger(), "Real path:   %zu steps", path_real.size());

        if (path_mapped.size() == path_real.size() && !path_mapped.empty())
            RCLCPP_INFO(get_logger(), "\n✓✓✓ SUCCESS: Mapped route is OPTIMAL! ✓✓✓\n");
        else if (path_mapped.empty())
            RCLCPP_ERROR(get_logger(), "\n✗ FAILED: No path found in mapped maze\n");
        else
            RCLCPP_WARN(get_logger(), "\n⚠ SUBOPTIMAL: Paths differ\n");
    }

    /**
     * Obtém o mapa real do serviço /get_map
     */
    std::vector<std::vector<std::string>> get_real_map()
    {
        auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();
        auto future = map_client_->async_send_request(request);

        auto start = std::chrono::steady_clock::now();
        while (rclcpp::ok())
        {
            rclcpp::spin_some(this->get_node_base_interface());

            if (future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready)
            {
                auto response = future.get();
                int rows = response->occupancy_grid_shape[0];
                int cols = response->occupancy_grid_shape[1];

                std::vector<std::vector<std::string>> grid(rows, std::vector<std::string>(cols));
                for (int i = 0; i < rows; i++)
                    for (int j = 0; j < cols; j++)
                        grid[i][j] = response->occupancy_grid_flattened[i * cols + j];
                return grid;
            }

            if (std::chrono::steady_clock::now() - start > std::chrono::seconds(5))
                return {};

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return {};
    }

    /**
     * Encontra posições do robô e alvo em um grid
     */
    void find_positions_in_grid(const std::vector<std::vector<std::string>> &grid,
                                Position &robot, Position &target)
    {
        for (size_t i = 0; i < grid.size(); i++)
            for (size_t j = 0; j < grid[i].size(); j++)
            {
                if (grid[i][j] == "r")
                    robot = Position(i, j);
                if (grid[i][j] == "t")
                    target = Position(i, j);
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
