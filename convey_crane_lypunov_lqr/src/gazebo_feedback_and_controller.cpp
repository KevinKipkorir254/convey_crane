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
#define RED_TEXT "\033[0;31m"
#define RESET_COLOR "\033[0m"

// Additional colors
#define BLACK_TEXT "\033[0;30m"
#define YELLOW_TEXT "\033[0;33m"
#define MAGENTA_TEXT "\033[0;35m"
#define CYAN_TEXT "\033[0;36m"
#define WHITE_TEXT "\033[0;37m"

// Bright colors
#define BRIGHT_BLACK_TEXT "\033[0;90m"
#define BRIGHT_RED_TEXT "\033[0;91m"
#define BRIGHT_GREEN_TEXT "\033[0;92m"
#define BRIGHT_YELLOW_TEXT "\033[0;93m"
#define BRIGHT_BLUE_TEXT "\033[0;94m"
#define BRIGHT_MAGENTA_TEXT "\033[0;95m"
#define BRIGHT_CYAN_TEXT "\033[0;96m"
#define BRIGHT_WHITE_TEXT "\033[0;97m"

// Background colors
#define BLACK_BACKGROUND "\033[40m"
#define RED_BACKGROUND "\033[41m"
#define GREEN_BACKGROUND "\033[42m"
#define YELLOW_BACKGROUND "\033[43m"
#define BLUE_BACKGROUND "\033[44m"
#define MAGENTA_BACKGROUND "\033[45m"
#define CYAN_BACKGROUND "\033[46m"
#define WHITE_BACKGROUND "\033[47m"

// Bright background colors
#define BRIGHT_BLACK_BACKGROUND "\033[100m"
#define BRIGHT_RED_BACKGROUND "\033[101m"
#define BRIGHT_GREEN_BACKGROUND "\033[102m"
#define BRIGHT_YELLOW_BACKGROUND "\033[103m"
#define BRIGHT_BLUE_BACKGROUND "\033[104m"
#define BRIGHT_MAGENTA_BACKGROUND "\033[105m"
#define BRIGHT_CYAN_BACKGROUND "\033[106m"
#define BRIGHT_WHITE_BACKGROUND "\033[107m"

// Text styles
#define BOLD_TEXT "\033[1m"
#define UNDERLINE_TEXT "\033[4m"
#define REVERSED_TEXT "\033[7m"

using std::placeholders::_1;
using namespace std::chrono_literals;

class JointStateSubscriber : public rclcpp::Node
{
public:
    JointStateSubscriber()
        : Node("state_space_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&JointStateSubscriber::process_the_input, this, _1));
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("effort_controller/commands", 10);

        this->declare_parameter("K", rclcpp::PARAMETER_DOUBLE_ARRAY);
        this->declare_parameter("ke", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("kv", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("kx", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("kdelta", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("lqr_transition_angle", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("initial_force", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("rviz_test", rclcpp::PARAMETER_BOOL);

        try
        {
            K_ = Eigen::Vector4d(this->get_parameter("K").as_double_array().data());
            ke_ = this->get_parameter("ke").as_double();
            kv_ = this->get_parameter("kv").as_double();
            kx_ = this->get_parameter("kx").as_double();
            kdelta_ = this->get_parameter("kdelta").as_double();
            lqr_transition_angle_ = convert_to_rads(this->get_parameter("lqr_transition_angle").as_double());
            initial_force_ = this->get_parameter("initial_force").as_double();
            rviz_test = this->get_parameter("rviz_test").as_bool();

            RCLCPP_WARN(this->get_logger(), "K: %.4f, %.4f, %.4f, %.4f", K_(0), K_(1), K_(2), K_(3));
            RCLCPP_WARN(this->get_logger(), "ke: %.4f", ke_);
            RCLCPP_WARN(this->get_logger(), "kv: %.4f", kv_);
            RCLCPP_WARN(this->get_logger(), "kx: %.4f", kx_);
            RCLCPP_WARN(this->get_logger(), "kdelta: %.4f", kdelta_);
            RCLCPP_WARN(this->get_logger(), "lqr_transition_angle_: %.4f", lqr_transition_angle_);
            RCLCPP_WARN(this->get_logger(), "lqr_transition_angle_: %.4f", convert_to_degrees(lqr_transition_angle_));
            RCLCPP_WARN(this->get_logger(), "initial_force_ : %.4f", initial_force_);
            if (rviz_test)
            {
                RCLCPP_WARN(this->get_logger(), "RVIZ_TEST: TRUE");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "RVIZ_TEST: FALSE");
            }
        }
        catch (const rclcpp::exceptions::ParameterUninitializedException &e)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Required parameter not defined: " << e.what());
            throw e;
        }
    }

private:
    void topic_callback(const sensor_msgs::msg::JointState &msg)
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
            if (!rviz_test)
            {
                slider_velocity = msg.velocity[index];
            }
            else
            {
                slider_velocity = slider_position - slider_previous_position;
                slider_previous_position = slider_position;
            }

            // RCLCPP_INFO(this->get_logger(), "Slider  -> Position: %.2f, Velocity: %.2f", slider_position, slider_velocity);
        }

        else
        {
            RCLCPP_WARN(this->get_logger(), "Slider joint not found in the message.");
        }

        if (swinger_it != msg.name.end())
        {
            auto index = std::distance(msg.name.begin(), swinger_it);
            swinger_position = msg.position[index];
            if (!rviz_test)
            {
                swinger_velocity = msg.velocity[index];
            }
            else
            {
                swinger_velocity = swinger_position - swinger_previous_position;
                swinger_previous_position = swinger_position;
            }

            // RCLCPP_INFO(this->get_logger(), "Swinger -> Position: %.2f, Velocity: %.2f", swinger_position, swinger_velocity);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Swinger joint not found in the message.");
        }

        double torque;

        if ((slider_it != msg.name.end()) || (swinger_it != msg.name.end()))
        {
            double converted_swinger_angle = change_angle_origin(swinger_position);
            double normalize_swinger_angle = normalize_to_pi(converted_swinger_angle); // this is to be used only for the LQR, otherwise use the above
            double converted_swinger_velocity = -1 * swinger_velocity;                      // increases velocity anti_clokwise

            double converted_slider_position = -1 * slider_position; // the direction to the right is what is positive
            double converted_slider_velocity = -1 * slider_velocity; // thus the velocity should also be inverted

            // RCLCPP_WARN(this->get_logger(), BRIGHT_GREEN_TEXT "Swinger -> %.4f slider1 -> %.4f vel1 -> %.4f vel2 -> %.4f", convert_to_degrees(converted_swinger_angle), converted_slider_position, convert_to_degrees(converted_swinger_velocity), converted_slider_velocity);

            // choosing a great transition angle
            double top_position = convert_to_rads(0);                  // the top is zero
            double lower_limit = top_position - lqr_transition_angle_; //-30
            double upper_limit = top_position + lqr_transition_angle_; // 30

            if (((normalize_swinger_angle) > (lower_limit)) && ((normalize_swinger_angle) < (upper_limit))) /// greater than -30 less than 30
            {
                torque = calculate_LQR_torque_output(converted_slider_position, converted_slider_velocity, normalize_swinger_angle, converted_swinger_velocity); // I think I can just use this as a regulato and do not require the error term
                RCLCPP_WARN(this->get_logger(), BRIGHT_MAGENTA_TEXT "Swinger -> %.4f slider1 -> %.4f vel1 -> %.4f vel2 -> %.4f T -> %.4f", convert_to_degrees(normalize_swinger_angle), converted_slider_position, convert_to_degrees(converted_swinger_velocity), converted_slider_velocity, torque);
            }
            else
            {
                torque = calculate_controller_force_output(converted_slider_position, converted_slider_velocity, converted_swinger_angle, converted_swinger_velocity);
                torque = -1 * torque;
                RCLCPP_WARN(this->get_logger(), BRIGHT_GREEN_TEXT "Swinger -> %.4f slider1 -> %.4f vel1 -> %.4f vel2 -> %.4f T -> %.4f", convert_to_degrees(converted_swinger_angle), converted_slider_position, convert_to_degrees(converted_swinger_velocity), converted_slider_velocity, torque);
                //RCLCPP_WARN(this->get_logger(), BRIGHT_GREEN_TEXT "Swinger_g -> %.4f Swinger_m -> %.4f  Swinger_n -> %.4f Swinger_vg -> %.4f Swinger_vm -> %.4f", convert_to_degrees(swinger_position), convert_to_degrees(converted_swinger_angle),  convert_to_degrees(normalize_swinger_angle), swinger_velocity, converted_swinger_velocity);
                //RCLCPP_WARN(this->get_logger(), BRIGHT_GREEN_TEXT "Slider_g -> %.4f Slider_m -> %.4f Slider_vg -> %.4f Slider_vm -> %.4f", slider_position, converted_slider_position, slider_velocity, converted_slider_velocity);
            }

            // Also positive force will have to be inverted normalize the angle
        }

        auto message = std_msgs::msg::Float64MultiArray();
        message.data = {torque}; //{ torque, 0.0};
        publisher_->publish(message);
    }

    void initialization(const sensor_msgs::msg::JointState &msg)
    {
        /*------------------------------INITIALIZATION PROCCESS--------------------------------------*/
        // Find the indices of the "slider" and "swinger" joints
        auto swinger_it = std::find(msg.name.begin(), msg.name.end(), "continuous_revolute_1");

        double swinger_position;
        double swinger_velocity;

        if (swinger_it != msg.name.end())
        {
            auto index = std::distance(msg.name.begin(), swinger_it);
            swinger_position = msg.position[index];
            if (!rviz_test)
            {
                swinger_velocity = msg.velocity[index];
            }
            else
            {
                swinger_velocity = swinger_position - swinger_previous_position;
                swinger_previous_position = swinger_position;
            }

            swinger_position = std::round(swinger_position * 1e5) / 1e5;
            swinger_velocity = std::round(swinger_velocity * 1e5) / 1e5;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Swinger joint not found in the message.");
        }

        if ((swinger_position == 0.0) && (swinger_velocity == 0.0))
        {
            RCLCPP_WARN(this->get_logger(), BOLD_TEXT "THE SWINGER IS AT PI AND THUS WE WILL FAIL TO REACH THE TOP");
            auto message = std_msgs::msg::Float64MultiArray();
            message.data = {initial_force_}; //{ 0.0, initial_force_};
            publisher_->publish(message);
            if (rviz_test)
            {
                initialize_ = true;
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), BOLD_TEXT "P: %.5f, V: %.5f", convert_to_degrees(swinger_position), convert_to_degrees(swinger_velocity));
            initialize_ = true;
        }
    }

    /**
     * @brief Processes the incoming joint state message.
     *
     * This function checks if the system has been initialized. If not, it calls the initialization function.
     * If the system has been initialized, it calls the topic callback function to process the joint state message.
     *
     * @param msg The incoming joint state message to be processed.
     */
    void process_the_input(const sensor_msgs::msg::JointState &msg)
    {
        if (!initialize_)
        {
            initialization(msg);
        }
        else
        {
            topic_callback(msg);
        }
    }

    double normalize_to_pi(double angle)
    {
        double normalized_position = fmod(angle, 2 * M_PI); // Wrap to [-2π, 2π]
        if (normalized_position > M_PI)
        {
            normalized_position -= 2 * M_PI; // Wrap to [-π, π]
        }
        else if (normalized_position < -M_PI)
        {
            normalized_position += 2 * M_PI; // Wrap to [-π, π]
        }
        return normalized_position;
    }

    /**
     * @brief Calculates the torque output for the LQR controller.
     *
     * This function computes the torque output for the LQR controller based on the given state variables and control parameters.
     * The torque is calculated using the LQR formulation with the provided error, slider position, slider velocity, and swinger velocity.
     *
     * @param slider_position The position of the cart (m).
     * @param slider_velocity The velocity of the cart (m/s).
     * @param swinger_position The angle of the pendulum (rad).
     * @param swinger_velocity The angular velocity of the pendulum (rad/s).
     * @param error The error between the current state and the desired state for the LQR controller.
     *
     * @return The torque output for the LQR controller (N*m).
     */
    double calculate_LQR_torque_output(double slider_position, double slider_velocity, double swinger_position, double swinger_velocity)
    {
        double gains_[4] = {K_(0), K_(1), K_(2), K_(3)};

        double error_gain = (gains_[2] * swinger_position); // since the centre position is zero, it can be used as a regulator
        double other_gains = (gains_[0] * slider_position) + (gains_[1] * slider_velocity) + (gains_[3] * swinger_velocity);

        double final_gain = (error_gain + other_gains); // this has to be check out and thought throug, something is not going great.
        RCLCPP_WARN(this->get_logger(), BRIGHT_MAGENTA_TEXT "K [ %.4f %.4f %.4f %.4f] IN [ %.4f %.4f %.4f %.4f] OUT [ %.4f %.4f %.4f]", gains_[0], gains_[1], gains_[2], gains_[3], slider_position, slider_velocity, swinger_position, swinger_velocity, error_gain, other_gains, final_gain);
        return final_gain;
    }

    double calculate_controller_force_output(double slider_position, double slider_velocity, double swinger_position, double swinger_velocity)
    {
        // Define system parameters
        double M = 2.7582989593438584919;                           // Mass of cart (kg)
        double m = 0.96730709424135585817 + 0.67500000000000004441; // Mass of pendulum (kg)
        double l = 0.5;                                             // Length of pendulum (m)
        double g = 9.81;                                            // Gravitational acceleration (m/s^2)

        // State variables
        double x = slider_position; // Cart position (m)
        /// subracting pi from the input to invert it
        double theta = swinger_position;  // Pendulum angle (rad)
        double dx = slider_velocity;      // Cart velocity (m/s)
        double dtheta = swinger_velocity; // Pendulum angular velocity (rad/s)

        // Mass matrix M(q)
        Eigen::Matrix2d Mq;
        Mq << M + m, m * l * cos(theta),
            m * l * cos(theta), m * l * l;

        // Coriolis/Centrifugal forces
        Eigen::Matrix2d Cq;
        Cq << 0, -m * l * sin(theta) * dtheta,
            0, 0;

        // Gravity vector G(q)
        Eigen::Vector2d Gq;
        Gq << 0,
            -m * g * l * sin(theta);

        // Control parameters
        /*--------------------------------------------------------------------------------*/
        // THIS SHOULD BE ADDED TO A YAML FILE
        double ke = ke_;         // reduce this since it is on the denominator
        double kv = kv_;         // leave this constant since it is both in numerator and denominator
        double kx = kx_;         // increase to make more aggressive since it is on the numerator
        double kdelta = kdelta_; // increase since it is at the numerator
        /*-------------------------------------------------------------------------------*/

        // Forming z
        double z3 = cos(theta);

        // forming zeds
        Eigen::Vector2d zeds;
        zeds << dx, dtheta;

        // Energy (E)
        double E = 0.5 * zeds.transpose() * Mq * zeds + m * g * l * (z3 - 1);

        // Numerator
        double numerator = kv * m * sin(theta) * (g * cos(theta) - l * dtheta * dtheta) - (M + m * sin(theta) * sin(theta)) * (kx * x + kdelta * dx);
        double denominator = kv + (M + m * sin(theta) * sin(theta)) * ke * E;

        // Control input
        double f = numerator / denominator;
        return f;
    }

    /**
     * @brief Converts an angle from radians to degrees.
     *
     * This function takes an angle in radians and converts it to degrees.
     * The conversion is performed using the formula: degrees = radians * (180 / PI).
     *
     * @param rads The angle in radians to be converted.
     * @return The angle in degrees.
     */
    double convert_to_degrees(double rads)
    {
        return (rads * 180 / M_PI);
    }

    /**
     * @brief Converts an angle from degrees to radians.
     *
     * This function takes an angle in degrees and converts it to radians.
     * The conversion is performed using the formula: radians = degrees * (PI / 180).
     *
     * @param degrees The angle in degrees to be converted.
     * @return The angle in radians.
     */
    double convert_to_rads(double degrees)
    {
        return (degrees * M_PI / 180);
    }

    double change_angle_origin(double angle)
    {
        return (convert_to_rads(180) - angle);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_; // effort controller publisher
    Eigen::Vector4d K_;
    double ke_;
    double kv_;
    double kx_;
    double kdelta_;
    double lqr_transition_angle_;
    bool initialize_ = false;
    bool rviz_test = true;
    double initial_force_;

    double swinger_previous_position = 0.0;
    double slider_previous_position = 0.0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateSubscriber>());
    rclcpp::shutdown();
    return 0;
}