#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "custom_interface/srv/average_vel.hpp"
#include <memory>

class UINode : public rclcpp::Node{
    public:
    UINode(): Node("ui_node"){
        RCLCPP_INFO(this->get_logger(), "UI Node has been started.");
        //pub, sub, service set up
        vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("bridge_vel", 10);
        //lambda per i service mi sembra più chiaro 
        avg_service = this->create_service<custom_interface::srv::AverageVel>(
            "average_velocity",
            [this] (const std::shared_ptr<custom_interface::srv::AverageVel::Request> request,
                    std::shared_ptr<custom_interface::srv::AverageVel::Response> response)
            {
                avgServiceCallBack(response);
            }
        );


        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&UINode::bridgeVelCallback, this));
        timer_->cancel(); //stop timer
        publishTime = 1; // tempo di pubblicazione in secondi
        isPublishing = false;
    }
    void runNode(){
        while(rclcpp::ok()){
            if(!isPublishing){
                getInput();
                isPublishing = true;
                startingTime = this->now();
                timer_->reset(); //restart timer
            }
            else {
                rclcpp::spin_some(this->get_node_base_interface());
            }
        }

    }

    void getInput(){
        geometry_msgs::msg::Twist inputTwist;
        std::cout << "Insert Linear Vel: " << std::endl;
        std::cin >> inputTwist.linear.x;
        std::cout << "Insert Angular Vel: " << std::endl;
        std::cin >> inputTwist.angular.z;
        vel_commands.push_back(inputTwist);
        RCLCPP_INFO(this->get_logger(), "Velocity command added: linear.x=%f, angular.z=%f", inputTwist.linear.x, inputTwist.angular.z);
        if(vel_commands.size() > 5){
            vel_commands.erase(vel_commands.begin());
        }
        isPublishing = true;


    }
    //PUB CALLBACK
    void bridgeVelCallback(){
        //time
        rclcpp::Time now = this->now();
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = vel_commands.back().linear.x;
        message.angular.z = vel_commands.back().angular.z;
        RCLCPP_INFO(this->get_logger(), "Publishing velocity command: linear.x=%f, angular.z=%f", message.linear.x, message.angular.z);
        
        //invio se sono sotto il secondo 
        if((now - startingTime).seconds() < publishTime){
            vel_publisher->publish(message);
        }
        else{
            //stop timer
            timer_->cancel();
            isPublishing = false;
            RCLCPP_INFO(this->get_logger(), "Finished publishing velocity commands.");
        }

    }
    //AVGVEL SERVICE CALLBACK
    void avgServiceCallBack(std::shared_ptr<custom_interface::srv::AverageVel::Response> response){
        float avg_lin = 0.0;
        float avg_ang = 0.0;
        for(const auto& cmd : vel_commands){
            avg_lin += cmd.linear.x;
            avg_ang += cmd.angular.z;
        }
        if(!vel_commands.empty()){
            avg_lin /= vel_commands.size();
            avg_ang /= vel_commands.size();
        }
        response->avg_lin = avg_lin;
        response->avg_ang = avg_ang;
        RCLCPP_INFO(this->get_logger(), "Average velocity calculated: avg_lin=%f, avg_ang=%f", avg_lin, avg_ang);
    }


    private:
    //Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;

    //Service 
    rclcpp::Service<custom_interface::srv::AverageVel>::SharedPtr avg_service;

    //Var
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<geometry_msgs::msg::Twist> vel_commands;
    float publishTime;
    bool isPublishing;
    rclcpp::Time startingTime;



};

int main( int argc, char **argv){
    rclcpp::init(argc, argv);
    std::shared_ptr<UINode> node = std::make_shared<UINode>();
    node ->runNode();
    return 0;
}