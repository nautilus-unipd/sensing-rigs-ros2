/*
 *
 */

#ifndef NORMALIZATOR_NODE_HPP
#define NORMALIZATOR_NODE_HPP

#include <string>

#include "rclcpp/qos.hpp"
#include "rmw/qos_profiles.h"
#include "std_msgs/msg/string.hpp"
#include "modem_msgs/msg/mono_ir.hpp"
#include "modem_msgs/msg/stereo_ir.hpp"
#include "modem_msgs/msg/stereo_vo.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class NormalizatorNode : public rclcpp_lifecycle::LifecycleNode
{
    public:
        /**
         * Class constants
         */
		static constexpr size_t QOS_DEPTH_PUB {5};
		static constexpr size_t QOS_DEPTH_SUB {5};
        const std::string TN_MODEM {"/to/modem"};
        const std::string TN_MONO_IR {"/output/cv_mono_ir"};
        const std::string TN_STEREO_IR {"/output/cv_stereo_ir"};
        const std::string TN_STEREO_VO {"/output/cv_stereo_vo"};
        const std::string PARAM_NAME_DEBUG {"debug"};
        const std::string PARAM_DESCR_DEBUG {"Prints additional debug informations."};

        /**
         * Class default constructor
         */
        explicit NormalizatorNode(std::string);

        /**
         *
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State&);

        /**
         *
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State&);

        /**
         *
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&);

        /**
         *
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&);

        /**
         *
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State&);

    private:

        /**
         *
         */
        void init_param(void);

        /**
         *
         */
        void init_var(void);

        /**
         * Class variables
         */
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr pub_json_;
        rclcpp::Subscription<modem_msgs::msg::MonoIR>::SharedPtr sub_mono_ir_;
        rclcpp::Subscription<modem_msgs::msg::StereoIR>::SharedPtr sub_stereo_ir_;
        rclcpp::Subscription<modem_msgs::msg::StereoVO>::SharedPtr sub_stereo_vo_;
        bool debug;
};

#endif // NORMALIZATOR_NODE_HPP
