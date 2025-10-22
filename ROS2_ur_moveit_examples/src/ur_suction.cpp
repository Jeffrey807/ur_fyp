#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <ur_msgs/srv/set_io.hpp>

#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstdlib>
#include <thread>
#include <chrono>

namespace
{
char read_key_nonblock()
{
  termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  char ch = 0;
  timeval tv {0, 0};
  fd_set rfds; FD_ZERO(&rfds); FD_SET(STDIN_FILENO, &rfds);
  if (select(STDIN_FILENO + 1, &rfds, nullptr, nullptr, &tv) > 0)
  {
    if (read(STDIN_FILENO, &ch, 1) <= 0) ch = 0;
  }
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}
}

class URSuctionNode : public rclcpp::Node
{
public:
  URSuctionNode()
  : Node("ur_suction")
  {
    // Parameters
    declare_parameter<int>("io_pin", 0);
    declare_parameter<std::string>("io_service", "/io_and_status_controller/set_io");
    
    io_pin_ = get_parameter("io_pin").as_int();
    io_service_ = get_parameter("io_service").as_string();

    // IO client
    io_client_ = create_client<ur_msgs::srv::SetIO>(io_service_);

    RCLCPP_INFO(get_logger(), "UR Suction Control Node Ready.");
    RCLCPP_INFO(get_logger(), "IO Service: %s", io_service_.c_str());
    RCLCPP_INFO(get_logger(), "IO Pin: %d", io_pin_);
    RCLCPP_INFO(get_logger(), "Commands:");
    RCLCPP_INFO(get_logger(), "  O - Turn suction ON (HIGH)");
    RCLCPP_INFO(get_logger(), "  F - Turn suction OFF (LOW)");
    RCLCPP_INFO(get_logger(), "  S - Test service connection");
    RCLCPP_INFO(get_logger(), "  Q - Quit");
    
    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&URSuctionNode::spin_keypoll, this));
  }

private:
  void spin_keypoll()
  {
    char k = read_key_nonblock();
    if (k == 'o' || k == 'O')
    {
      set_suction(true);
    }
    else if (k == 'f' || k == 'F')
    {
      set_suction(false);
    }
    else if (k == 's' || k == 'S')
    {
      test_service_connection();
    }
    else if (k == 'q' || k == 'Q')
    {
      RCLCPP_INFO(get_logger(), "Shutting down...");
      rclcpp::shutdown();
    }
  }

  bool set_suction(bool high)
  {
    if (!io_client_->wait_for_service(std::chrono::seconds(2)))
    {
      RCLCPP_ERROR(get_logger(), "IO service not available: %s", io_service_.c_str());
      return false;
    }

    auto req = std::make_shared<ur_msgs::srv::SetIO::Request>();
    req->fun = ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
    req->pin = static_cast<uint8_t>(io_pin_);
    req->state = high ? 1.0 : 0.0;

    RCLCPP_INFO(get_logger(), "Setting IO pin %d to %s", io_pin_, high ? "HIGH" : "LOW");

    auto fut = io_client_->async_send_request(req);
    auto ret = fut.wait_for(std::chrono::seconds(2));
    
    if (ret == std::future_status::ready)
    {
      auto response = fut.get();
      RCLCPP_INFO(get_logger(), "IO command sent successfully. Response: %s", 
                  response->success ? "SUCCESS" : "FAILED");
      return response->success;
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "IO command timeout");
      return false;
    }
  }

  void test_service_connection()
  {
    RCLCPP_INFO(get_logger(), "Testing connection to IO service: %s", io_service_.c_str());
    
    if (io_client_->wait_for_service(std::chrono::seconds(2)))
    {
      RCLCPP_INFO(get_logger(), "✓ IO service is available");
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "✗ IO service is NOT available");
      RCLCPP_INFO(get_logger(), "Make sure the UR robot driver is running and the service is active");
    }
  }

  int io_pin_ {0};
  std::string io_service_;
  rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr io_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<URSuctionNode>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
