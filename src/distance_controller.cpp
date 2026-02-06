#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"   
#include "custom_interface/msg/info_robot.hpp"
#include <memory>
#include <limits>

enum Direction{
    FRONT,
    BACK,
    LEFT,
    RIGHT,
};

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

        info_publisher = this->create_publisher<custom_interface::msg::InfoRobot>("info", 10);


        //Timer Testing
        info_timer = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&ControllerNode::infoCallBack, this)
        );


        outputVel = geometry_msgs::msg::Twist();
        minDistance =   std::numeric_limits<float>::infinity(); //set to max
        collisionDetected = false;
        
        



    }
    private:

    void laserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "Laser scan received with %zu ranges.", msg->ranges.size());
        //process laser data
        
        //filter invalid data 
        std::vector<float> valid_ranges;
        for(size_t i = 0; i < msg->ranges.size(); i++){
            float range = msg -> ranges[i];
            if(range >= msg->range_min && range <= msg->range_max){
                valid_ranges.push_back(range);
                angle = msg->angle_min + i * msg->angle_increment;
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

        //last input command check to prevent robot from moving after comingBack
        bool isStopCommand = (msg->linear.x == 0.0 && msg->angular.z == 0.0);
        if(isStopCommand){
            collisionDetected = false;
        }


        //checking collision
        if(minDistance > threshold && !collisionDetected){
            outputVel.linear.x = msg->linear.x;
            outputVel.angular.z = msg->angular.z;
            RCLCPP_INFO(this->get_logger(), "Publishing velocity command: linear.x=%f, angular.z=%f", outputVel.linear.x, outputVel.angular.z);
            vel_publisher->publish(outputVel);
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "COLLISION DETECTED!");
            collisionDetected = true;
            comeBack(msg);
            
        }

    }

    void infoCallBack(){
        custom_interface::msg::InfoRobot info_msg;
        info_msg.distance = minDistance;
        info_msg.tresh = threshold;
        info_msg.direction = getDirection(angle);
        info_publisher->publish(info_msg);
    }

    std::string getDirection(const double& angle){
        if(angle >= -0.785 && angle <= 0.785){
            return "FRONT";
        }
        else if(angle > 0.785 && angle <= 2.356){
            return "LEFT";
        }
        else if(angle < -0.785 && angle >= -2.356){
            return "RIGHT";
        }
        else{
            return "BACK";
        }

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
    rclcpp::Publisher<custom_interface::msg::InfoRobot>::SharedPtr info_publisher;
   
    rclcpp::TimerBase::SharedPtr info_timer;
    geometry_msgs::msg::Twist outputVel;
    float threshold  = 0.5;
    float minDistance ;
    double angle;
    bool collisionDetected;
    

};

int main( int argc, char **argv){
    rclcpp::init(argc, argv);
    std::shared_ptr<ControllerNode> node = std::make_shared<ControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}