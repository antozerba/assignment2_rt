#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"   
#include <memory>
#include <limits>

class ControllerNode : public rclcpp::Node{
    public:
    ControllerNode(): Node("controller_node"){
        RCLCPP_INFO(this->get_logger(), "Distance Controller Node has been started.");

        //Initialization Components
        vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        laser_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&ControllerNode::laserCallBack, this, std::placeholders::_1));
        ui_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "bridge_vel", 10, std::bind(&ControllerNode::uiCallBack, this, std::placeholders::_1));

        //Timer Testing
        // vel_timer = this->create_wall_timer(
        //     std::chrono::milliseconds(20), std::bind(&ControllerNode::velCallBack, this)
        // );

        outputVel = geometry_msgs::msg::Twist();
        minDistance =   std::numeric_limits<float>::infinity(); //set to max
        
        



    }
    private:

    void laserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "Laser scan received with %zu ranges.", msg->ranges.size());
        //process laser data
        
        //filter invalid data 
        std::vector<float> valid_ranges;
        for(const float& range : msg->ranges){
            if(range >= msg->range_min && range <= msg->range_max){
                valid_ranges.push_back(range);
            }
        }
        //find min distance
        if (!valid_ranges.empty()) {
            minDistance = *std::min_element(valid_ranges.begin(), valid_ranges.end());
        }
        RCLCPP_INFO(this->get_logger(), "Minimum distance to obstacle: %f", minDistance);
    }


    void uiCallBack(const geometry_msgs::msg::Twist::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "UI command received: linear.x=%f, angular.z=%f", msg->linear.x, msg->angular.z);

        //checking collision
        if(minDistance > threshold){
            outputVel.linear.x = msg->linear.x;
            outputVel.angular.z = msg->angular.z;
            RCLCPP_INFO(this->get_logger(), "Publishing velocity command: linear.x=%f, angular.z=%f", outputVel.linear.x, outputVel.angular.z);
            vel_publisher->publish(outputVel);
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "COLLISION DETETED!");
            comeBack(msg);
            
        }



    }


    void velCallBack(){
        outputVel.linear.x = 1.0;
        vel_publisher->publish(outputVel);
    }

    void comeBack(const geometry_msgs::msg::Twist::SharedPtr msg){
        geometry_msgs::msg::Twist reverse_cmd;
        reverse_cmd.linear.x = -msg->linear.x;
        reverse_cmd.angular.z = -msg->angular.z;
        RCLCPP_ERROR(this->get_logger(), "Obstacle too close! Reversing: linear.x=%f, angular.z=%f", reverse_cmd.linear.x, reverse_cmd.angular.z);   
        vel_publisher -> publish(reverse_cmd);

    }

    //Components
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ui_subscriber;
   
    // rclcpp::TimerBase::SharedPtr vel_timer; //for testing
    geometry_msgs::msg::Twist outputVel;
    float threshold  = 0.5;
    float minDistance ;

    






};

int main( int argc, char **argv){
    rclcpp::init(argc, argv);
    std::shared_ptr<ControllerNode> node = std::make_shared<ControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}