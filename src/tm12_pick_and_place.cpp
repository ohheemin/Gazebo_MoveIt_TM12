#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>

#include <thread>
#include <chrono>
#include <vector>
#include <cmath>
#include <mutex>
#include <cstdlib>  

using namespace std::chrono_literals;

enum class J1Direction
{
    ANY,
    INCREASE,
    DECREASE
};

enum class SequenceState
{
    IDLE,
    PICKING,
    PICK_TO_PLACE,
    PLACING,
    PLACE_RETURNING,
    COMPLETED
};

class MoveItToJointCommand : public rclcpp::Node
{
public:
    MoveItToJointCommand() : Node("moveit_to_jointcommand")
    {
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_command", 10);

        pickplace_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/pickandplace", 10);

        placepick_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/placeandpick", 10);

        finish_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/finish", 10);

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            10,
            std::bind(&MoveItToJointCommand::jointStateCallback, this, std::placeholders::_1));

        pickplace_timer_ = this->create_wall_timer(
            60ms, std::bind(&MoveItToJointCommand::publishJoint1, this));

        finish_timer_ = this->create_wall_timer(
            60ms, std::bind(&MoveItToJointCommand::publishFinish, this));

        cmd_timer_ = this->create_wall_timer(
            60ms, std::bind(&MoveItToJointCommand::periodicCommandPublish, this));

        RCLCPP_INFO(this->get_logger(),
            "MoveIt Bridge Node Started. Subscribing to /joint_states");
    }

    void initialize()
    {
        move_group_ =
            std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), "tmr_arm");

        move_group_->setPlanningPipelineId("ompl");
        move_group_->setPlannerId("RRTConnectkConfigDefault");
        move_group_->setPlanningTime(10.0);
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.5);

        waitForFirstJointState();

        {
            std::lock_guard<std::mutex> lock(joint_mutex_);
            std::lock_guard<std::mutex> cmd_lock(command_mutex_);
            last_commanded_joints_ = current_joints_;
            has_command_ = true;
        }

        RCLCPP_INFO(this->get_logger(), "Initializing: Clearing Gripper State...");
        releaseObject(); 
        std::this_thread::sleep_for(1s);

        tf2::Quaternion q;
        q.setRPY(M_PI, 0, 0);

        geometry_msgs::msg::Pose pick_pose1;
        pick_pose1.position.x = 0.3+0.65;
        pick_pose1.position.y = -0.7+4.7 ;
        pick_pose1.position.z = 0.275+0.63;
        pick_pose1.orientation = tf2::toMsg(q);

        geometry_msgs::msg::Pose pick_approach1 = pick_pose1;
        pick_approach1.position.z += 0.3;

        //place poses 일 때 항상 밑 방향으로 보는 constraint 적용 시 경로계산 실패-------------->>> 다시 체크

        geometry_msgs::msg::Pose place_pose1;
        place_pose1.position.x = 0.0 + 0.6;
        place_pose1.position.y = 0.5 + 4.8;
        place_pose1.position.z = 0.43 + 0.63; //0.4+0.63+0.15
        place_pose1.orientation = tf2::toMsg(q);

        geometry_msgs::msg::Pose place_approach1 = place_pose1;
        place_approach1.position.z += 0.1;

        geometry_msgs::msg::Pose place_pose2;
        place_pose2.position.x = 0.235 + 0.6;
        place_pose2.position.y = 0.5 + 4.8;
        place_pose2.position.z = 0.43 + 0.63; //0.4+0.63 
        place_pose2.orientation = tf2::toMsg(q);
        geometry_msgs::msg::Pose place_approach2 = place_pose2;
        place_approach2.position.z += 0.1;

        geometry_msgs::msg::Pose place_pose3;
        place_pose3.position.x = 0.555 + 0.6;
        place_pose3.position.y = 0.5 + 4.8;
        place_pose3.position.z = 0.43 + 0.63; //0.4+0.63
        place_pose3.orientation = tf2::toMsg(q);
        geometry_msgs::msg::Pose place_approach3 = place_pose3;
        place_approach3.position.z += 0.1;

        RCLCPP_WARN(this->get_logger(), "Pick Place Sequence Started");

        // CYCLE 1
        RCLCPP_INFO(this->get_logger(), "Starting: PICK 1");
        current_sequence_state_ = SequenceState::PICK_TO_PLACE;
        run_sequence(pick_approach1, J1Direction::ANY);
        std::this_thread::sleep_for(7s);

        current_sequence_state_ = SequenceState::PICKING;
        if (!move_to_pose_smart(pick_pose1))
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to move to pick_pose1");
            return;
        }
        std::this_thread::sleep_for(3s);
        
        // Pick 전에 혹시 모를 연결 해제
        releaseObject(); 
        std::this_thread::sleep_for(500ms);
        gripObject();
        std::this_thread::sleep_for(4s);

        current_sequence_state_ = SequenceState::PICK_TO_PLACE;
        if (!move_to_pose_smart(pick_approach1))
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to move back to pick_approach1");
            return;
        }
        std::this_thread::sleep_for(7s);

        RCLCPP_INFO(this->get_logger(), "Starting: PLACE 1");
        run_sequence(place_approach1, J1Direction::ANY);
        std::this_thread::sleep_for(7s);

        current_sequence_state_ = SequenceState::PLACING;
        if (!move_to_pose_smart(place_pose1))
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to move to place_pose1");
            return;
        }
        std::this_thread::sleep_for(3s);
        releaseObject();
        std::this_thread::sleep_for(4s);

        current_sequence_state_ = SequenceState::PLACE_RETURNING;
        if (!move_to_pose_smart(place_approach1))
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to move back to place_approach1");
            return;
        }
        std::this_thread::sleep_for(7s);

        // --- CYCLE 2 ---
        RCLCPP_INFO(this->get_logger(), "Starting: PICK 2");
        run_sequence(pick_approach1, J1Direction::ANY);
        std::this_thread::sleep_for(7s);

        current_sequence_state_ = SequenceState::PICKING;
        if (!move_to_pose_smart(pick_pose1))
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to move to pick_pose1");
            return;
        }
        std::this_thread::sleep_for(3s);
        
        // Pick 전에 혹시 모를 연결 해제
        // releaseObject(); 
        std::this_thread::sleep_for(0s);
        gripObject();
        std::this_thread::sleep_for(4s);

        current_sequence_state_ = SequenceState::PICK_TO_PLACE;
        if (!move_to_pose_smart(pick_approach1))
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to move back to pick_approach1");
            return;
        }
        std::this_thread::sleep_for(7s);

        RCLCPP_INFO(this->get_logger(), "Starting: PLACE 2");
        run_sequence(place_approach2, J1Direction::ANY);
        std::this_thread::sleep_for(7s);

        current_sequence_state_ = SequenceState::PLACING;
        if (!move_to_pose_smart(place_pose2))
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to move to place_pose2");
            return;
        }
        std::this_thread::sleep_for(3s);
        releaseObject();
        std::this_thread::sleep_for(4s);

        current_sequence_state_ = SequenceState::PLACE_RETURNING;
        if (!move_to_pose_smart(place_approach2))
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to move back to place_approach2");
            return;
        }
        std::this_thread::sleep_for(7s);

        // --- CYCLE 3 ---
        RCLCPP_INFO(this->get_logger(), "Starting: PICK 3");
        run_sequence(pick_approach1, J1Direction::ANY);
        std::this_thread::sleep_for(7s);

        current_sequence_state_ = SequenceState::PICKING;
        if (!move_to_pose_smart(pick_pose1))
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to move to pick_pose1");
            return;
        }
        std::this_thread::sleep_for(3s);
        
        // Pick 전에 혹시 모를 연결 해제
        releaseObject(); 
        std::this_thread::sleep_for(500ms);
        gripObject();
        std::this_thread::sleep_for(4s);

        current_sequence_state_ = SequenceState::PICK_TO_PLACE;
        if (!move_to_pose_smart(pick_approach1))
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to move back to pick_approach1");
            return;
        }
        std::this_thread::sleep_for(7s);

        RCLCPP_INFO(this->get_logger(), "Starting: PLACE 3");
        run_sequence(place_approach3, J1Direction::ANY);
        std::this_thread::sleep_for(7s);

        current_sequence_state_ = SequenceState::PLACING;
        if (!move_to_pose_smart(place_pose3))
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to move to place_pose3");
            return;
        }
        std::this_thread::sleep_for(3s);
        releaseObject();
        std::this_thread::sleep_for(4s);

        current_sequence_state_ = SequenceState::PLACE_RETURNING;
        if (!move_to_pose_smart(place_approach3))
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to move back to place_approach3");
            return;
        }
        std::this_thread::sleep_for(7s);

        current_sequence_state_ = SequenceState::COMPLETED;
        RCLCPP_WARN(this->get_logger(), "All Pick & Place Completed");
        
        RCLCPP_INFO(this->get_logger(), "Sending all joints to zero position");
        std::vector<double> zero_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        publish_joint_command(zero_joints);
        
        sequence_finished_ = true;
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pickplace_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr placepick_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finish_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::TimerBase::SharedPtr pickplace_timer_;
    rclcpp::TimerBase::SharedPtr finish_timer_;
    rclcpp::TimerBase::SharedPtr cmd_timer_;

    std::mutex joint_mutex_;
    std::mutex command_mutex_;
    std::vector<double> current_joints_;
    std::vector<double> last_commanded_joints_; 
    bool have_joint_state_ = false;
    bool has_command_ = false; 
    bool sequence_finished_ = false;
    SequenceState current_sequence_state_ = SequenceState::IDLE;

    void gripObject()
    {
        RCLCPP_INFO(this->get_logger(), "Gripper: ATTACHING object via ign topic");

        std::this_thread::sleep_for(100ms);
        int result = system("ign topic -t /tm12_gripper/attach -m ignition.msgs.Empty -p \"\"");
        if (result != 0)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to execute attach command");
        }
    }

    void releaseObject()
    {
        RCLCPP_INFO(this->get_logger(), "Gripper: DETACHING object via ign topic");
        
        std::this_thread::sleep_for(100ms);
        int result = system("ign topic -t /tm12_gripper/detach -m ignition.msgs.Empty -p \"\"");
        if (result != 0)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to execute detach command");
        }
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        if (msg->position.size() >= 6)
        {
            current_joints_ = msg->position;
            have_joint_state_ = true;
        }
    }

    void periodicCommandPublish()
    {
        std::vector<double> joints_to_pub;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            if (!has_command_) return;
            joints_to_pub = last_commanded_joints_;
        }

        sensor_msgs::msg::JointState msg;
        msg.name = {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};
        msg.position = joints_to_pub;
        msg.header.stamp = this->now();
        joint_pub_->publish(msg);
    }

    void publishJoint1()
    {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        if (!have_joint_state_) return;

        double j1 = current_joints_[0];
        std_msgs::msg::Float64 msg;
        msg.data = j1;

        if (current_sequence_state_ == SequenceState::PLACING || 
            current_sequence_state_ == SequenceState::PLACE_RETURNING)
        {
            if ((j1 >= 4.92 && j1 <= 4.94) ||
                (j1 >= 4.55 && j1 <= 4.70) ||
                (j1 >= 4.21 && j1 <= 4.23))
            {
                placepick_pub_->publish(msg);
                return; 
            }
        }

        pickplace_pub_->publish(msg);
    }

    void publishFinish()
    {
        std_msgs::msg::Bool msg;
        msg.data = sequence_finished_;
        finish_pub_->publish(msg);
    }

    void waitForFirstJointState()
    {
        rclcpp::Rate rate(10);
        while (rclcpp::ok() && !have_joint_state_)
            rate.sleep();
    }

    bool checkJointConstraints(const std::vector<double>& joints, const std::string& context = "")
    {
        if (joints.size() < 4) 
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid joint vector size: %zu", joints.size());
            return false;
        }
        
        double j1 = joints[0];
        double j2 = joints[1];
        double j3 = joints[2];
        double j4 = joints[3];
        
        if (j1 < -0.1 || j2 < -0.4 || j3 < -0.1 || j4 > 0.1)
        {
            RCLCPP_ERROR(this->get_logger(), 
                "%sConstraint violation! j1=%.3f (≥0.0), j2=%.3f (≥-0.4), j3=%.3f (≥0.0), j4=%.3f (≤0.0)",
                context.empty() ? "" : (context + ": ").c_str(),
                j1, j2, j3, j4);
            return false;
        }
        return true;
    }

    bool move_to_pose_with_retry(const geometry_msgs::msg::Pose& target)
    {
        move_group_->setStartStateToCurrentState();
        move_group_->setPoseTarget(target);
        
        for (int attempt = 0; attempt < 50; attempt++)
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_WARN(this->get_logger(), "Pose planning attempt %d: Planning failed", attempt + 1);
                continue;
            }

            bool valid = true;
            for (size_t i = 0; i < plan.trajectory_.joint_trajectory.points.size(); ++i)
            {
                if (!checkJointConstraints(
                    plan.trajectory_.joint_trajectory.points[i].positions, 
                    "Pose planning point " + std::to_string(i)))
                {
                    valid = false;
                    break;
                }
            }

            if (valid)
            {
                RCLCPP_INFO(this->get_logger(), "Pose planning attempt %d: Valid trajectory found!", attempt + 1);
                playTrajectory(plan.trajectory_.joint_trajectory);
                return true;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Pose planning attempt %d: Constraint violation", attempt + 1);
            }
        }
        
        RCLCPP_ERROR(this->get_logger(), "Failed to find valid trajectory after 50 attempts");
        return false;
    }

    bool move_to_pose_smart(const geometry_msgs::msg::Pose& target)
    {
        move_group_->setStartStateToCurrentState();
        std::vector<geometry_msgs::msg::Pose> waypoints{target};
        moveit_msgs::msg::RobotTrajectory traj;
        
        double fraction = move_group_->computeCartesianPath(waypoints, 0.005, 1.5, traj);
        
        if (fraction > 0.95)
        {
            bool valid = true;
            for (size_t i = 0; i < traj.joint_trajectory.points.size(); ++i)
            {
                if (!checkJointConstraints(traj.joint_trajectory.points[i].positions,
                                          "Cartesian planning point " + std::to_string(i)))
                {
                    valid = false;
                    break;
                }
            }
            
            if (valid)
            {
                RCLCPP_INFO(this->get_logger(), "Using Cartesian path (%.2f%% complete)", fraction * 100.0);
                playTrajectory(traj.joint_trajectory);
                return true;
            }
        }

        RCLCPP_WARN(this->get_logger(), "Cartesian failed (%.2f%%), trying regular planning", fraction * 100.0);
        return move_to_pose_with_retry(target);
    }

    std::vector<double> plan_and_publish(
        const geometry_msgs::msg::Pose& pose,
        J1Direction j1_dir)
    {
        move_group_->setStartStateToCurrentState();
        move_group_->setPoseTarget(pose);

        double start_j1 = current_joints_[0];

        for (int attempt = 0; attempt < 100; attempt++)
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (move_group_->plan(plan) !=
                moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_WARN(this->get_logger(), "Attempt %d: Planning failed", attempt + 1);
                continue;
            }

            bool valid = true;
            for (size_t i = 0; i < plan.trajectory_.joint_trajectory.points.size(); ++i)
            {
                auto& p = plan.trajectory_.joint_trajectory.points[i];

                if (!checkJointConstraints(p.positions, "Planning point " + std::to_string(i)))
                {
                    valid = false;
                    break;
                }

                double j1 = p.positions[0];
                
                if (j1_dir == J1Direction::INCREASE && j1 < start_j1)
                {
                    valid = false;
                    break;
                }

                if (j1_dir == J1Direction::DECREASE && j1 > start_j1)
                {
                    valid = false;
                    break;
                }
            }

            if (!valid)
            {
                RCLCPP_WARN(this->get_logger(), "Attempt %d: Constraint violation", attempt + 1);
                continue;
            }

            RCLCPP_INFO(this->get_logger(), "Attempt %d: Valid trajectory found!", attempt + 1);

            playTrajectory(plan.trajectory_.joint_trajectory);
            return plan.trajectory_.joint_trajectory.points.back().positions;
        }

        RCLCPP_FATAL(this->get_logger(), "FAILED to find valid trajectory after 100 attempts");
        return {};
    }

    void run_sequence(const geometry_msgs::msg::Pose& pose, J1Direction dir)
    {
        plan_and_publish(pose, dir);
    }

    void playTrajectory(const trajectory_msgs::msg::JointTrajectory& traj)
    {
        RCLCPP_INFO(this->get_logger(), "Executing trajectory with %zu points", traj.points.size());
        
        for (size_t i = 0; i < traj.points.size(); ++i)
        {
            auto& p = traj.points[i];

            if (!checkJointConstraints(p.positions, "Execution point " + std::to_string(i)))
            {
                RCLCPP_FATAL(this->get_logger(), 
                    "Trajectory execution STOPPED at point %zu/%zu due to constraint violation!", 
                    i, traj.points.size());
                return;
            }
            
            publish_joint_command(p.positions);
            std::this_thread::sleep_for(60ms);
        }
        
        RCLCPP_INFO(this->get_logger(), "Trajectory execution completed successfully");
    }

    void publish_joint_command(const std::vector<double>& joints)
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        last_commanded_joints_ = joints;
        has_command_ = true;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveItToJointCommand>();

    std::thread([node]() {
        std::this_thread::sleep_for(3s);
        node->initialize();
    }).detach();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
