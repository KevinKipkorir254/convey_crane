#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include <stdio.h>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#define PI 3.14159

// ANSI escape codes for colors
#define GREEN_TEXT "\033[0;32m"
#define BLUE_TEXT "\033[0;34m"
#define RESET_COLOR "\033[0m"

using std::placeholders::_1;
using namespace std::chrono_literals;

class JointStateSubscriber : public rclcpp::Node
{
  public:
    JointStateSubscriber()
    : Node("state_space_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&JointStateSubscriber::topic_callback, this, _1));
      publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("effort_controller/commands", 10);

      this->declare_parameter("K", rclcpp::PARAMETER_DOUBLE_ARRAY);
      this->declare_parameter("ke", rclcpp::PARAMETER_DOUBLE);
      this->declare_parameter("kx", rclcpp::PARAMETER_DOUBLE);
      this->declare_parameter("yy", rclcpp::PARAMETER_DOUBLE);
      this->declare_parameter("lqr_transition_angle", rclcpp::PARAMETER_DOUBLE);

                        
            try {
                K_ = Eigen::Vector4d(this->get_parameter("K").as_double_array().data());
                ke_ = this->get_parameter("ke").as_double();
                kx_ = this->get_parameter("kx").as_double();
                y_ = this->get_parameter("yy").as_double();
                lqr_transition_angle_ = this->get_parameter("lqr_transition_angle").as_double();

                
            RCLCPP_WARN(this->get_logger(), "K: %.4f, %.4f, %.4f, %.4f", K_(0), K_(1), K_(2), K_(3));
            RCLCPP_WARN(this->get_logger(), "ke: %.4f", ke_);
            RCLCPP_WARN(this->get_logger(), "kv: %.4f", kx_);
            RCLCPP_WARN(this->get_logger(), "kx: %.4f", y_);
            RCLCPP_WARN(this->get_logger(), "lqr_transition_angle_: %.4f", lqr_transition_angle_);

            } catch (const rclcpp::exceptions::ParameterUninitializedException & e) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Required parameter not defined: " << e.what());
                throw e;
            }

    }

  private:
    void topic_callback(const sensor_msgs::msg::JointState & msg)
    {
        // Find the indices of the "slider" and "swinger" joints
        auto slider_it = std::find(msg.name.begin(), msg.name.end(), "slider_1");
        auto swinger_it = std::find(msg.name.begin(), msg.name.end(), "continuous_revolute_1");

        
            double slider_position;
            double slider_velocity;

            
            double swinger_position;
            double swinger_velocity;

            double switching_range = lqr_transition_angle_;


        if (slider_it != msg.name.end())
        {
            auto index = std::distance(msg.name.begin(), slider_it);
            slider_position = msg.position[index];
            slider_velocity = msg.velocity[index];

            //RCLCPP_INFO(this->get_logger(), "Slider  -> Position: %.2f, Velocity: %.2f", slider_position, slider_velocity);
        }

        else
        {
            RCLCPP_WARN(this->get_logger(), "Slider joint not found in the message.");
        }

        if (swinger_it != msg.name.end())
        {
            auto index = std::distance(msg.name.begin(), swinger_it);
            swinger_position = msg.position[index];
            swinger_velocity = msg.velocity[index];

            //RCLCPP_INFO(this->get_logger(), "Swinger -> Position: %.2f, Velocity: %.2f", swinger_position, swinger_velocity);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Swinger joint not found in the message.");
        }

        double f;

        if((slider_it != msg.name.end()) || (swinger_it != msg.name.end()))
        {
              // Define the range boundaries
              double upper_limit = PI + switching_range;
              double lower_limit = PI - switching_range;

                  // Normalize swinger_position to [0, 2*PI]
              double normalized_position = fmod(swinger_position, 2 * PI);

                  // If the normalized position is negative, adjust it by adding 2*PI
              if (normalized_position < 0) {
                  normalized_position += 2 * PI;
              }
                  
         //RCLCPP_INFO(this->get_logger(), "normalised_position: %.4f", normalized_position);

         //Before the pendulum is in the homoclinic orbit
        if( normalized_position < lower_limit || normalized_position > upper_limit)
        {

            // Control parameters
            /*--------------------------------------------------------------------------------*/
            //THIS SHOULD BE ADDED TO A YAML FILE
            double ke = ke_;  //reduce to mae the thing more agreessive
            double kx = kx_; //increase to make the thing more aggressive
            double y = y_;     //increase to mae the thing more aggressive
            /*-------------------------------------------------------------------------------*/
            
            double output = (-(1/ke))*(kx*slider_position + y*slider_velocity);
            f = output;
            RCLCPP_INFO(this->get_logger(), BLUE_TEXT"F: %.4f", f);
            //RCLCPP_INFO(this->get_logger(), "Force: %.4f Numerator: %.4f Denominator: %.4f Energy: %.4f", f, numerator, denominator, E);
        }

                    else 
                    {
                    double gains_[4] = { K_(0), K_(1), K_(2), K_(3)};

                    double from_output = (gains_[0]*swinger_position);
                    double first = (gains_[1]*swinger_velocity) + (gains_[2]*slider_position) +(gains_[3]*slider_velocity);

                    double final_gain = (from_output - first);
                    f = final_gain;
                    RCLCPP_INFO(this->get_logger(), GREEN_TEXT"F: %.4f", f);

                    }

                    auto message = std_msgs::msg::Float64MultiArray();
                    message.data.push_back(f);
                    publisher_->publish(message);
                        }
    }
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_; // effort controller publisher
    Eigen::Vector4d K_;
    double ke_;  //reduce to mae the thing more agreessive
    double kx_; //increase to make the thing more aggressive
    double y_;     //increase to mae the thing more aggressive
    double lqr_transition_angle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateSubscriber>());
  rclcpp::shutdown();
  return 0;
}