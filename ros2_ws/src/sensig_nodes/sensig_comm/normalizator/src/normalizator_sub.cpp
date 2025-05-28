/*
	TODO: results publisher (message already defined)
	TODO: remove '\t', '\n' and spaces from results, not needed by the modem
*/
#include <memory>

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/qos_profiles.h"
#include "custom_msgs/msg/mono_ir.hpp"
#include "custom_msgs/msg/odometry.hpp"
#include "custom_msgs/msg/stereo_ir.hpp"
#include "custom_msgs/msg/json_string.hpp"

class NormalizatorSub : public rclcpp::Node
{
    public:
        // CLASS CONSTANTs
        const std::string TN_MONO {"/output/cv_mono"};
        const std::string TN_ODOMETRY {"/output/cv_odometry"};
        const std::string TN_STEREO {"/output/cv_stereo"};
		size_t QOS_DEPTH {5};

        // CLASS DEFAULT CONTRUCTOR
        NormalizatorSub() : Node("normalizator_sub")
        {
            // initialize lambda functions
            auto callback_mono = [this](custom_msgs::msg::MonoIR::UniquePtr msg) -> void
            {
                std::string output = "\n{\n\t\"mono_ir\": {\n\t\t\"timestamp\": \"" + msg->timestamp + "\",\n\t\t\"label\": \"" + msg->label + "\",\n\t\t\"confidence\": \"" + std::to_string(msg->confidence) + "\",\n\t\t\"box\": {\"" + std::to_string(msg->box[0]) + "\", \"" + std::to_string(msg->box[1]) + "\", \"" + std::to_string(msg->box[2]) + "\", \"" + std::to_string(msg->box[3]) + "\"}\n\t}\n}\n";
                // DEBUG
                RCLCPP_INFO(this->get_logger(), "%s", output.c_str());
            };

            auto callback_odometry = [this](custom_msgs::msg::Odometry::UniquePtr msg) -> void
            {
                std::string output = "\n{\n\t\"odometry\": {\n\t\t\"timestamp\": \"" + msg->timestamp + "\",\n\t\t\"translation\": {\"" + std::to_string(msg->translation[0]) + "\", \"" + std::to_string(msg->translation[1]) + "\", \"" + std::to_string(msg->translation[2]) + "\"},\n\t\t\"rotation\": {\"" + std::to_string(msg->rotation[0]) + "\", \"" + std::to_string(msg->rotation[1]) + "\", \"" + std::to_string(msg->rotation[2]) + "\", \"" + std::to_string(msg->rotation[3]) + "\", \"" + std::to_string(msg->rotation[4]) + "\", \"" + std::to_string(msg->rotation[5]) + "\", \"" + std::to_string(msg->rotation[6]) + "\", \"" + std::to_string(msg->rotation[7]) + "\", \"" + std::to_string(msg->rotation[8]) + "\"}\n\t}\n}\n";
                // DEBUG
                RCLCPP_INFO(this->get_logger(), "%s", output.c_str());
            };

            auto callback_stereo = [this](custom_msgs::msg::StereoIR::UniquePtr msg) -> void
            {
                std::string output = "\n{\n\t\"stereo_ir\": {\n\t\t\"timestamp\": \"" + msg->timestamp + "\",\n\t\t\"label\": \"" + msg->label + "\"\n\t}\n}\n";
                // DEBUG
                RCLCPP_INFO(this->get_logger(), "%s", output.c_str());
            };

			// create QoS profile for subscribers
			// keep in memory last 5 messages (they small)
			// use default profile
			rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(this->QOS_DEPTH), rmw_qos_profile_default);

            // create subscription for every topic
            this->sub_mono_ = this->create_subscription<custom_msgs::msg::MonoIR>(this->TN_MONO, qos, callback_mono);
            this->sub_odometry_ = this->create_subscription<custom_msgs::msg::Odometry>(this->TN_ODOMETRY, qos, callback_odometry);
            this->sub_stereo_ = this->create_subscription<custom_msgs::msg::StereoIR>(this->TN_STEREO, qos, callback_stereo);
        }

    private:
        // CLASS VARIABLEs
        rclcpp::Subscription<custom_msgs::msg::MonoIR>::SharedPtr sub_mono_;
        rclcpp::Subscription<custom_msgs::msg::Odometry>::SharedPtr sub_odometry_;
        rclcpp::Subscription<custom_msgs::msg::StereoIR>::SharedPtr sub_stereo_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NormalizatorSub>());
    rclcpp::shutdown();
    return 0;
}
