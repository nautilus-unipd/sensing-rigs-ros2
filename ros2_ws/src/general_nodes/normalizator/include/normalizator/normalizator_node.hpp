/*
 * Definitions of costructors, methods, variables and constants
 * for the NormalizatorNode.
 * This node has the objective to convert some custom ROS 2
 * messages to a JSON format, stored in the classic string format,
 * which makes the job easier for the modem to parse and communicate
 * the results. It is an intermediary node between image elaboration
 * (performed by the package "cv_algorithms") and the modem.
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
         * Class default constructor, takes in input the name for the new node.
         */
        explicit NormalizatorNode(std::string);

        /**
         * Lifecycle node function called when the node is first instanced.
         * Called only once during the life time of the node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State&);

        /**
         * Lifecycle node function called to start / restore the running state
         * of the node, it initializes any active component.
         * Reverses the changes made by "on_activate".
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State&);

        /**
         * Lifecycle node function called when the node is paused due
         * to error or to the programmer choice, it stops any active component.
         * Reverses the changes made by "on_activate".
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&);

        /**
         * Lifecycle node function called before the node is killed
         * to prepare it before destruction.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&);

        /**
         * Lifecycle node function called when the node is killed,
         * it frees any resource allocated.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State&);

    private:

        /**
         * Function to parse the parameters given in input.
         */
        void init_param(void);

        /**
         * Function to initialize node's variables.
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
