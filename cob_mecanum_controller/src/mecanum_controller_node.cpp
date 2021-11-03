/*
 * Copyright 2020 Fraunhofer Institute for Manufacturing Engineering and
 * Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cob_mecanum_controller/mecanum_controller.h>

#include <cob_srvs/SetFloat.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <exception>

class MecanumControllerNode {
public:
  MecanumControllerNode() : nh_() {
    bool all_parameters_set = true;
    ros::NodeHandle pnh("~");

    bool use_dimensions_from_tf{false};
    pnh.getParam("use_dimensions_from_tf", use_dimensions_from_tf);

    auto get_param = [&pnh, &all_parameters_set](std::string const &key,
                                                 auto &target) {
      if (!pnh.getParam(key, target)) {
        ROS_ERROR_STREAM("Parameter " << key
                                      << " was not declared in the scope");
        all_parameters_set = false;
      }
    };
    get_param("r", r_);
    if (!use_dimensions_from_tf) {
      get_param("lx", lx_);
      get_param("ly", ly_);
    } else {
      get_param("front_left_frame", joint_frame_names_.at(0));
      get_param("front_right_frame", joint_frame_names_.at(1));
      get_param("rear_left_frame", joint_frame_names_.at(2));
    }

    get_param("front_left_joint_name", joint_names_.at(0));
    get_param("front_right_joint_name", joint_names_.at(1));
    get_param("rear_left_joint_name", joint_names_.at(2));
    get_param("rear_right_joint_name", joint_names_.at(3));

    if (!all_parameters_set) {
      throw std::runtime_error("At least one parameter is missing.");
    }

    static_frame_ = "map";
    pnh.getParam("static_frame", static_frame_);
    odom_frame_ = "map";
    pnh.getParam("odom_frame", odom_frame_);

    controller_ = std::make_shared<cob_mecanum_controller::MecanumController>(
        lx_, ly_, r_);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10, false);
    joint_cmd_pub_ =
        nh_.advertise<sensor_msgs::JointState>("wheel_command", 10, false);

    twist_sub_ = nh_.subscribe<geometry_msgs::Twist>(
        "cmd_vel", 1, &MecanumControllerNode::twistCallback, this);

    joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>(
        "wheel_state", 1, &MecanumControllerNode::jointStateCallback, this);

    auto makeCallback = [this](auto &&setter) {
      boost::function<bool(cob_srvs::SetFloatRequest &,
                           cob_srvs::SetFloatResponse &)>
          cb = [this,
                setter = std::move(setter)](cob_srvs::SetFloatRequest &req,
                                            cob_srvs::SetFloatResponse &res) {
            setter(req.data);
            controller_ =
                std::make_shared<cob_mecanum_controller::MecanumController>(
                    lx_, ly_, r_);
            res.message = "success";
            res.success = true;
            return true;
          };
      return cb;
    };
    if (!use_dimensions_from_tf) {
      set_lx_service_ = nh_.advertiseService(
          "set_lx", makeCallback([this](double value) { lx_ = value; }));
      set_ly_service_ = nh_.advertiseService(
          "set_ly", makeCallback([this](double value) { ly_ = value; }));
      set_r_service_ = nh_.advertiseService(
          "set_r", makeCallback([this](double value) { r_ = value; }));
    } else {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      boost::function<void(const ros::TimerEvent &)> cb = [this](auto const &) {
        auto front_left_to_front_right = tf_buffer_->lookupTransform(
            joint_frame_names_.at(0), joint_frame_names_.at(1), ros::Time(0));
        auto front_left_to_rear_left = tf_buffer_->lookupTransform(
            joint_frame_names_.at(1), joint_frame_names_.at(2), ros::Time(0));
        auto length_from_transform = [](auto const &t) -> double {
          return std::sqrt(std::pow(t.transform.translation.x, 2) +
                           std::pow(t.transform.translation.y, 2));
        };
        lx_ = length_from_transform(front_left_to_rear_left);
        ly_ = length_from_transform(front_left_to_front_right);
        controller_ =
            std::make_shared<cob_mecanum_controller::MecanumController>(
                lx_, ly_, r_);
      };
      timer_ = nh_.createTimer(ros::Duration(1.0), cb);
    }
  }

protected:
  ros::NodeHandle nh_;
  ros::Subscriber twist_sub_;
  ros::Subscriber joint_state_sub_;

  ros::Publisher odom_pub_;
  ros::Publisher joint_cmd_pub_;
  std::shared_ptr<cob_mecanum_controller::MecanumController> controller_;

  std::string static_frame_;
  std::string odom_frame_;

  ros::ServiceServer set_lx_service_;
  ros::ServiceServer set_ly_service_;
  ros::ServiceServer set_r_service_;
  double ly_{0};
  double lx_{0};
  double r_{0};
  std::array<std::string, 4> joint_names_;
  std::array<std::string, 3> joint_frame_names_;
  ros::Timer timer_;
  std::shared_ptr<tf2_ros::TransformListener>
      tf_listener_; ///< Updates the transform buffer
  std::shared_ptr<tf2_ros::Buffer>
      tf_buffer_; ///< TF Buffer used for zone->map and robot->map transforms

  void twistCallback(const geometry_msgs::Twist msg) {
    Eigen::Vector3d twist;
    twist << msg.linear.x, msg.linear.y, msg.angular.z;
    Eigen::Vector4d wheel_velocities = controller_->twistToWheel(twist);
    sensor_msgs::JointState joint_command;
    joint_command.name.resize(4);
    std::copy(joint_names_.begin(), joint_names_.end(),
              joint_command.name.begin());
    joint_command.velocity = std::vector<double>(
        wheel_velocities.data(),
        wheel_velocities.data() +
            wheel_velocities.rows() * wheel_velocities.cols());
    joint_cmd_pub_.publish(joint_command);
  }

  void jointStateCallback(const sensor_msgs::JointState msg) {
    Eigen::Vector4d wheel_velocities;
    auto get_by_name = [&msg](auto const& name){
      if (auto candidate = std::find(msg.name.begin(),msg.name.end(),name); candidate != msg.name.end()){
        return msg.velocity.at(std::distance(msg.name.begin(),candidate));
      }
      throw std::out_of_range("Could not find joint with name " +name);
    };
    try{
      for (size_t i = 0 ; i < joint_names_.size();++i){
        wheel_velocities(i) = get_by_name(joint_names_.at(i));
      }
    }
    catch(std::out_of_range const& e){
      ROS_ERROR_STREAM("Could not process joint state feedback. Original error " << e.what());
      return;
    }
    Eigen::Vector3d twist = controller_->wheelToTwist(wheel_velocities);
    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = static_frame_;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.child_frame_id = odom_frame_;

    odom_msg.twist.twist.linear.x = twist.x();
    odom_msg.twist.twist.linear.y = twist.y();
    odom_msg.twist.twist.angular.z = twist.z();
    odom_pub_.publish(odom_msg);
  }
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "mecanum_controller");
  MecanumControllerNode mcn;
  ros::spin();
}
