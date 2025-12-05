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
#include <map>

class Part2Mapper : public rclcpp::Node
{
public:
    Part2Mapper() : Node("part2_mapper"), current_pos_(0, 0)
    {
        rclcpp::QoS qos = rclcpp::SensorDataQoS();
        sensor_sub_ = create_subscription<cg_interfaces::msg::RobotSensors>(
            "/culling_games/robot_sensors", qos,
            [this](const cg_interfaces::msg::RobotSensors::ConstSharedPtr msg)
            {
                last_sensors_ = msg;
                sensor_received_ = true;
            });

        move_client_ = create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        get_map_client_ = create_client<cg_interfaces::srv::GetMap>("/get_map");
        while (!move_client_->wait_for_service(std::chrono::seconds(1)))
        {
        }

        timer_ = create_wall_timer(std::chrono::milliseconds(200),
                                   std::bind(&Part2Mapper::explore, this));

        visited_.insert({current_pos_.row, current_pos_.col});
        path_stack_.push(current_pos_);
    }

private:
    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensor_sub_;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr get_map_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    Mapper mapper_;
    Position current_pos_;
    bool sensor_received_ = false;
    std::set<std::pair<int, int>> visited_;
    std::stack<Position> path_stack_;
    cg_interfaces::msg::RobotSensors::ConstSharedPtr last_sensors_;

    void explore()
    {
        if (!sensor_received_)
            return;

        // Atualiza o mapa COM A POSIÇÃO ATUAL CORRETA
        if (last_sensors_)
        {
            std::map<std::string, std::string> sensors = {
                {"up", last_sensors_->up}, {"down", last_sensors_->down}, {"left", last_sensors_->left}, {"right", last_sensors_->right}, {"up_left", last_sensors_->up_left}, {"up_right", last_sensors_->up_right}, {"down_left", last_sensors_->down_left}, {"down_right", last_sensors_->down_right}};
            mapper_.update_map(current_pos_, sensors);
        }

        if (mapper_.target_found())
        {
            RCLCPP_INFO(get_logger(), "ALVO ENCONTRADO!");
            timer_->cancel();
            validate();
            return;
        }

        std::vector<std::pair<int, int>> dirs = {{0, 1}, {0, -1}, {-1, 0}, {1, 0}};
        std::vector<std::string> names = {"up", "down", "left", "right"};

        for (size_t i = 0; i < 4; i++)
        {
            Position next(current_pos_.row + dirs[i].first, current_pos_.col + dirs[i].second);

            if (!visited_.count({next.row, next.col}))
            {
                CellType cell = mapper_.get_cell(next);

                if (cell == CellType::FREE || cell == CellType::UNKNOWN)
                {
                    if (move(names[i]))
                    {
                        RCLCPP_INFO(get_logger(), "Move %s: (%d,%d)",
                                    names[i].c_str(), current_pos_.row, current_pos_.col);
                        visited_.insert({current_pos_.row, current_pos_.col});
                        path_stack_.push(current_pos_);
                    }
                    return;
                }
            }
        }

        if (path_stack_.size() > 1)
        {
            path_stack_.pop();
            Position target = path_stack_.top();

            int dr = target.row - current_pos_.row;
            int dc = target.col - current_pos_.col;

            RCLCPP_WARN(get_logger(), "BACK: (%d,%d)->(%d,%d)",
                        current_pos_.row, current_pos_.col, target.row, target.col);

            if (dc == 1)
                move("up");
            else if (dc == -1)
                move("down");
            else if (dr == -1)
                move("left");
            else if (dr == 1)
                move("right");
            return;
        }

        RCLCPP_INFO(get_logger(), "COMPLETO");
        timer_->cancel();
        validate();
    }

    bool move(const std::string &dir)
    {
        auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        req->direction = dir;
        auto future = move_client_->async_send_request(req);

        if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready)
        {
            auto resp = future.get();
            if (resp->success)
            {
                current_pos_ = Position(resp->robot_pos[0], resp->robot_pos[1]);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                return true;
            }
        }
        return false;
    }

    void validate()
    {
        auto real_grid = get_real_map();
        auto mapped_grid = mapper_.to_grid();
        Position robot_pos = mapper_.get_robot_position();
        Position target_pos = mapper_.get_target_position();

        Pathfinder pf_mapped(mapped_grid);
        auto path_mapped = pf_mapped.find_path_bfs(robot_pos, target_pos);

        Pathfinder pf_real(real_grid);
        Position r_real, t_real;
        find_pos(real_grid, r_real, t_real);
        auto path_real = pf_real.find_path_bfs(r_real, t_real);

        RCLCPP_INFO(get_logger(), "Mapeado:%zu Real:%zu", path_mapped.size(), path_real.size());

        if (path_mapped.size() == path_real.size() && !path_mapped.empty())
            RCLCPP_INFO(get_logger(), "✓ ROTA ÓTIMA!");
        else
            RCLCPP_WARN(get_logger(), "Diferente");
    }

    std::vector<std::vector<std::string>> get_real_map()
    {
        auto req = std::make_shared<cg_interfaces::srv::GetMap::Request>();
        auto future = get_map_client_->async_send_request(req);
        if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready)
            return {};

        auto resp = future.get();
        int rows = resp->occupancy_grid_shape[0];
        int cols = resp->occupancy_grid_shape[1];

        std::vector<std::vector<std::string>> grid(rows, std::vector<std::string>(cols));
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                grid[i][j] = resp->occupancy_grid_flattened[i * cols + j];
        return grid;
    }

    void find_pos(const std::vector<std::vector<std::string>> &grid,
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
    rclcpp::spin(std::make_shared<Part2Mapper>());
    rclcpp::shutdown();
    return 0;
}
