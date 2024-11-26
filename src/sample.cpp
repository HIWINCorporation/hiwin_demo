#include <functional>  // For std::bind and placeholders
#include <signal.h>    // For signal handling
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

class SampleNode : public rclcpp::Node
{
public:
  SampleNode(std::string controller_name) : Node("sample_node")
  {
    // Declare and retrieve the robot description parameter
    this->declare_parameter<std::string>("robot_description", "");
    std::string robot_description = this->get_parameter("robot_description").as_string();
    if (robot_description.empty())
    {
      RCLCPP_FATAL(this->get_logger(), "robot_description parameter is empty. Please check your setup.");
      rclcpp::shutdown();
      return;
    }

    // Create a kinematic tree and extract the kinematic chain from base to tool
    kdl_parser::treeFromString(robot_description, robot_tree_);
    robot_tree_.getChain("base_link", "tool0", chain_);

    // Initialize KDL solvers for forward and inverse velocity kinematics
    fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
    ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain_, 0.0000001);

    // Extract joint names from the kinematic chain
    for (size_t i = 0; i < chain_.getNrOfSegments(); i++)
    {
      auto joint = chain_.getSegment(i).getJoint();
      if (joint.getType() != KDL::Joint::Fixed)
      {
        trajectory_msg_.joint_names.push_back(joint.getName());
      }
    }

    // Resize trajectory points to match the number of joints
    trajectory_point_msg_.positions.resize(chain_.getNrOfJoints());
    trajectory_point_msg_.velocities.resize(chain_.getNrOfJoints());

    // Initialize motion-related flags and tolerances
    is_motion_done_ = true;
    tolerance_ = 0.001;
    velocity_tolerance_ = 0.0;
    target_joint_names_ = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
    joint_positions_.resize(chain_.getNrOfJoints(), 0.0);

    // Create publisher for joint trajectories
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(controller_name + "/joint_trajectory",
                                                                               rclcpp::SystemDefaultsQoS());
    // Subscribe to joint state messages
    jnt_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SystemDefaultsQoS(),
        std::bind(&SampleNode::joint_state_cb, this, std::placeholders::_1));

    // Subscribe to controller state messages
    state_sub_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        controller_name + "/controller_state", rclcpp::SystemDefaultsQoS(),
        std::bind(&SampleNode::controller_state_cb, this, std::placeholders::_1));
  }

  // Command the robot to move to a specified joint position
  void go_point(const std::vector<double>& positions, const rclcpp::Duration time_from_start)
  {
    std::memcpy(trajectory_point_msg_.positions.data(), positions.data(),
                trajectory_point_msg_.positions.size() * sizeof(double));
    trajectory_point_msg_.time_from_start = time_from_start;

    trajectory_msg_.points.clear();
    trajectory_msg_.points.push_back(trajectory_point_msg_);

    publisher_->publish(trajectory_msg_);
    RCLCPP_INFO(this->get_logger(), "Go to position A1: %f A2: %f A3: %f A4: %f A5: %f A6: %f ",
                trajectory_point_msg_.positions[0], trajectory_point_msg_.positions[1],
                trajectory_point_msg_.positions[2], trajectory_point_msg_.positions[3],
                trajectory_point_msg_.positions[4], trajectory_point_msg_.positions[5]);
  }

  // Stop all motion and hold the current joint positions
  void stop_motion()
  {
    std::memcpy(trajectory_point_msg_.positions.data(), joint_positions_.data(),
                trajectory_point_msg_.positions.size() * sizeof(double));
    trajectory_point_msg_.time_from_start = rclcpp::Duration::from_seconds(0.1);

    trajectory_msg_.points.clear();
    trajectory_msg_.points.push_back(trajectory_point_msg_);

    publisher_->publish(trajectory_msg_);
    RCLCPP_INFO(this->get_logger(), "Current position A1: %f A2: %f A3: %f A4: %f A5: %f A6: %f ",
                trajectory_point_msg_.positions[0], trajectory_point_msg_.positions[1],
                trajectory_point_msg_.positions[2], trajectory_point_msg_.positions[3],
                trajectory_point_msg_.positions[4], trajectory_point_msg_.positions[5]);
    RCLCPP_INFO(this->get_logger(), "Published trajectory to hold current position");
  }

  // Generate a cyclic trajectory and send it to the robot
  void test_cyclic(const std::vector<double>& start_positions, const rclcpp::Duration time_from_start)
  {
    auto twist = KDL::Twist();  // End-effector twist
    double total_time = time_from_start.seconds();
    int trajectory_len = 360;
    int loop_rate = trajectory_len / total_time;
    double dt = 1.0 / loop_rate;
    KDL::JntArray joint_positions = KDL::JntArray(chain_.getNrOfJoints());
    KDL::JntArray joint_velocities = KDL::JntArray(chain_.getNrOfJoints());

    std::memcpy(joint_positions.data.data(), start_positions.data(), start_positions.size() * sizeof(double));

    trajectory_msg_.points.clear();
    for (int i = 0; i < trajectory_len; i++)
    {
      // set endpoint twist
      double t = i;
      twist.vel.x(0.15 * cos(2 * M_PI * t / trajectory_len));
      twist.vel.y(0.15 * sin(2 * M_PI * t / trajectory_len));

      // convert cart to joint velocities
      ik_vel_solver_->CartToJnt(joint_positions, twist, joint_velocities);

      // copy to trajectory_point_msg
      std::memcpy(trajectory_point_msg_.positions.data(), joint_positions.data.data(),
                  trajectory_point_msg_.positions.size() * sizeof(double));
      std::memcpy(trajectory_point_msg_.velocities.data(), joint_velocities.data.data(),
                  trajectory_point_msg_.velocities.size() * sizeof(double));

      // integrate joint velocities
      joint_positions.data += joint_velocities.data * dt;

      // set timing information
      trajectory_point_msg_.time_from_start.sec = i / loop_rate;
      trajectory_point_msg_.time_from_start.nanosec = static_cast<int>(
          1E9 / loop_rate * static_cast<double>(t - loop_rate * (i / loop_rate)));  // implicit integer division

      trajectory_msg_.points.push_back(trajectory_point_msg_);
    }

    publisher_->publish(trajectory_msg_);
  }

  // Check if motion is complete
  bool is_motion_done()
  {
    return is_motion_done_;
  }

  // Check if the node has been initialized
  bool initialized()
  {
    return initialized_;
  }

private:
  void joint_state_cb(const sensor_msgs::msg::JointState& msg)
  {
    if (chain_.getNrOfJoints() != msg.name.size())
    {
      RCLCPP_ERROR_ONCE(this->get_logger(), "Received wrong joint data. (ex. robot type error)");
      return;
    }

    std::unordered_map<std::string, size_t> name_to_index;
    for (size_t i = 0; i < msg.name.size(); ++i)
    {
      name_to_index[msg.name[i]] = i;
    }

    for (size_t i = 0; i < target_joint_names_.size(); ++i)
    {
      auto it = name_to_index.find(target_joint_names_[i]);
      if (it != name_to_index.end())
      {
        joint_positions_[i] = msg.position[it->second];
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Joint %s not found in joint_states!", target_joint_names_[i].c_str());
      }
    }

    if (!initialized_)
    {
      RCLCPP_INFO(this->get_logger(), "Initialize position A1: %f A2: %f A3: %f A4: %f A5: %f A6: %f ",
                  joint_positions_.at(0), joint_positions_.at(1), joint_positions_.at(2), joint_positions_.at(3),
                  joint_positions_.at(4), joint_positions_.at(5));
      initialized_ = true;
    }
  }

  void controller_state_cb(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
  {
    bool motion_done = true;

    for (size_t i = 0; i < msg->error.positions.size(); ++i)
    {
      double position_error = std::abs(msg->error.positions[i]);
      double velocity_error = std::abs(msg->error.velocities[i]);

      if (position_error > tolerance_ || velocity_error > velocity_tolerance_)
      {
        motion_done = false;
        break;
      }
    }
    is_motion_done_ = motion_done;
  }

  // Publishers and subscribers
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jnt_sub_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr state_sub_;

  // Kinematic tree and solvers
  KDL::Tree robot_tree_;
  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;

  // Trajectory message
  trajectory_msgs::msg::JointTrajectory trajectory_msg_;
  trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_msg_;

  // Motion control flags and parameters
  bool initialized_;
  bool is_motion_done_;
  double tolerance_;
  double velocity_tolerance_;
  std::vector<std::string> target_joint_names_;
  std::vector<double> joint_positions_;
};

static std::shared_ptr<SampleNode> node;

// Signal handler for graceful shutdown
void signal_handler(int signum)
{
  if (signum == SIGINT)
  {
    node->stop_motion();
    rclcpp::shutdown();
    exit(1);
  }
}

int main(int argc, char** argv)
{
  signal(SIGINT, signal_handler);  // Register signal handler

  rclcpp::init(argc, argv);

  rclcpp::Duration time_from_start(0, 0);
  std::vector<double> position{ KDL::deg2rad * 0.0, KDL::deg2rad * 45.0,  KDL::deg2rad * -45.0,
                                KDL::deg2rad * 0.0, KDL::deg2rad * -90.0, KDL::deg2rad * 0.0 };
  int step = 0;

  if (argc > 2 && strcmp("--controller_name", argv[1]) == 0)
  {
    node = std::make_shared<SampleNode>(std::string(argv[2]));
  }
  else
  {
    rclcpp::shutdown();
    return -1;
  }

  // Main loop to control motion in steps
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);

    switch (step)
    {
      case 0:
        if (node->initialized())
        {
          step++;
        }
        break;

      case 1:
        time_from_start = rclcpp::Duration::from_seconds(5);
        node->go_point(position, time_from_start);
        step++;
        break;

      case 2:
        if (!node->is_motion_done())
        {
          step++;
        }
        break;

      case 3:
        if (node->is_motion_done())
        {
          step++;
        }
        break;

      case 4:
        time_from_start = rclcpp::Duration::from_seconds(5);
        node->test_cyclic(position, time_from_start);
        step++;
        break;

      case 5:
        if (!node->is_motion_done())
        {
          step++;
        }
        break;

      case 6:
        if (node->is_motion_done())
        {
          step = 4;
        }
        break;
    };

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();
  return 0;
}
