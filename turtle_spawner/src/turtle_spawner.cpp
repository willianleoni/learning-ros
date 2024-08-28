#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include <cstdlib>  // Para std::rand
#include <ctime>    // Para std::time

using namespace std::chrono_literals;

class TurtleSpawner : public rclcpp::Node
{
public:
    TurtleSpawner() : Node("turtle_spawner"), turtle_count_(1)
    {
        // Inicializa o cliente do serviço 'spawn'
        client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
        
        // Configura o timer para chamar spawn_turtle a cada 10 segundos
        timer_ = this->create_wall_timer(10s, std::bind(&TurtleSpawner::spawn_turtle, this));
        
        // Inicializa o gerador de números aleatórios
        std::srand(std::time(nullptr));
    }

void spawn_turtle()
{
    RCLCPP_INFO(this->get_logger(), "Attempting to spawn a turtle");

    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    request->x = static_cast<float>(std::rand() % 11);
    request->y = static_cast<float>(std::rand() % 11);
    request->theta = 0.0;
    request->name = "spawned_turtle" + std::to_string(turtle_count_++);

    while (!client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Client interrupted while waiting for service to appear.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for spawn service to be available...");
    }

    auto future = client_->async_send_request(request);

    try
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Spawned turtle: %s", response->name.c_str());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to spawn turtle: %s", e.what());
    }
}

    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    int turtle_count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Cria o nó TurtleSpawner
    auto node = std::make_shared<TurtleSpawner>();

    // Mantém o nó ativo
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
