#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <yolo_msgs/msg/detection_array.hpp>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("main_task_node_logger");
using namespace std::chrono_literals;
namespace mtc = moveit::task_constructor;

class SprpiMainTaskNode
{
public: 

	SprpiMainTaskNode(const rclcpp::NodeOptions& options);
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
	void initPlanners();
	void depthDetectionsCallback(const yolo_msgs::msg::DetectionArray::SharedPtr msg);
	void mirrorDetectionsCallback(const yolo_msgs::msg::DetectionArray::SharedPtr msg);
	void loopCallback();
    mtc::Task createPickTask(const std::string object_id);
    mtc::Task createPlaceTask(bool is_diseased, const std::string object_id);
	void doTask();

private:

    struct IKParameters {
        int64_t angle_delta;
        double min_solution_distance;
        int64_t max_ik_solutions;
        double jump_threshold;
    };

	struct ArmStates {
		bool is_picking = false;
		bool is_infront_of_mirror = false;
		std::string picked_object = "";
        mtc::Task current_task;
	};

	
	ArmStates arm_states_;
    IKParameters ik_params_;

    rclcpp::Node::SharedPtr node_;
	std::string depth_detections_topic_name_;
    std::string mirror_detections_topic_name_;
    rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr depth_detections_subscription_;
    rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr mirror_detections_subscription_;
    rclcpp::TimerBase::SharedPtr loop_;
	bool is_diseased_;
    yolo_msgs::msg::DetectionArray depth_detections_;
    yolo_msgs::msg::DetectionArray mirror_detections_;

    std::string arm_group_name_ = "arm";
    std::string hand_group_name_ = "Gripper";
    std::string hand_frame_ = "gripper_base";

    std::shared_ptr<mtc::solvers::PipelinePlanner> sampling_planner_;
    std::shared_ptr<mtc::solvers::JointInterpolationPlanner> interpolation_planner_;
    std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner_;

	std::set<std::string> previous_object_ids_;
};

SprpiMainTaskNode::SprpiMainTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("main_task_node", options) }
{
    // Task planning parameters
    ik_params_.angle_delta = node_->get_parameter("angle_delta").as_int();
	RCLCPP_INFO_ONCE(LOGGER, "angle_delta is set to: %ld", ik_params_.angle_delta);
    ik_params_.min_solution_distance = node_->get_parameter("min_solution_distance").as_double();
	RCLCPP_INFO_ONCE(LOGGER, "min_solution_distance is set to: %f", ik_params_.min_solution_distance);
    ik_params_.max_ik_solutions = node_->get_parameter("max_ik_solutions").as_int();
	RCLCPP_INFO_ONCE(LOGGER, "max_ik_solutions is set to: %ld", ik_params_.max_ik_solutions);
    ik_params_.jump_threshold = node_->get_parameter("jump_threshold").as_double();
	RCLCPP_INFO_ONCE(LOGGER, "jump_threshold is set to: %f", ik_params_.jump_threshold);

    // Robot description parameters
    arm_group_name_ = node_->get_parameter("arm_group_name_").as_string();
	RCLCPP_INFO_ONCE(LOGGER, "arm_group_name_ is set to: %s", arm_group_name_.c_str());
    hand_group_name_ = node_->get_parameter("hand_group_name_").as_string();
	RCLCPP_INFO_ONCE(LOGGER, "hand_group_name_ is set to: %s", hand_group_name_.c_str());
    hand_frame_ = node_->get_parameter("hand_frame_").as_string();
	RCLCPP_INFO_ONCE(LOGGER, "hand_frame_ is set to: %s", hand_frame_.c_str());

	depth_detections_topic_name_ = node_->get_parameter("depth_detections_topic_name_").as_string();
	RCLCPP_INFO_ONCE(LOGGER, "depth_detections_topic_name_ is set to: %s", depth_detections_topic_name_.c_str());
    mirror_detections_topic_name_ = node_->get_parameter("mirror_detections_topic_name_").as_string();
	RCLCPP_INFO_ONCE(LOGGER, "mirror_detections_topic_name_ is set to: %s", mirror_detections_topic_name_.c_str());

    depth_detections_subscription_= node_->create_subscription<yolo_msgs::msg::DetectionArray>(
        depth_detections_topic_name_, 
        10, 
        std::bind(&SprpiMainTaskNode::depthDetectionsCallback, this, std::placeholders::_1)
    );

    mirror_detections_subscription_= node_->create_subscription<yolo_msgs::msg::DetectionArray>(
        mirror_detections_topic_name_, 
        10, 
        std::bind(&SprpiMainTaskNode::mirrorDetectionsCallback, this, std::placeholders::_1)
    );

	loop_ = node_->create_wall_timer(20ms, std::bind(&SprpiMainTaskNode::loopCallback, this));
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr SprpiMainTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void SprpiMainTaskNode::initPlanners()
{
    sampling_planner_ = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    interpolation_planner_ = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    cartesian_planner_ = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner_->setMaxVelocityScalingFactor(1.0);
    cartesian_planner_->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner_->setStepSize(.01);
    cartesian_planner_->setJumpThreshold(ik_params_.jump_threshold);
}

void SprpiMainTaskNode::depthDetectionsCallback(const yolo_msgs::msg::DetectionArray::SharedPtr msg)
{   
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
	std::set<std::string> current_object_ids;
	collision_objects.reserve(msg->detections.size() + 1);
	if (arm_states_.picked_object != "") {
        current_object_ids.insert(arm_states_.picked_object);
    }

    for (const auto &detection : msg->detections) {
		auto id = detection.id;
		auto class_name = detection.class_name;
		auto frame_id =  detection.bbox3d.frame_id;
		auto dimensions = detection.bbox3d.size;
        dimensions.z = (dimensions.x+dimensions.y)/2;
		auto pose = detection.bbox3d.center;

		moveit_msgs::msg::CollisionObject object;
		object.id = id + "_" + class_name;
		object.header.frame_id = frame_id;
		object.primitives.resize(2);
        object.primitive_poses.resize(2);
		object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
		object.primitives[0].dimensions = { dimensions.x, dimensions.y, dimensions.z };
        object.primitives[1].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        object.primitives[1].dimensions = { 0.05, 0.003 };
        object.primitive_poses[0].position.x= 0.0;
        object.primitive_poses[0].position.y= ((dimensions.z+0.05)*0.5);
        object.primitive_poses[0].position.z= 0.0;
        object.primitive_poses[1].orientation.w= 0.7071;
        object.primitive_poses[1].orientation.x= 0.7071;
        object.primitive_poses[1].orientation.y= 0.0;
        object.primitive_poses[1].orientation.z= 0.0;
		object.pose = pose;

        if (class_name == "ripe" || class_name == "diseased") {
            current_object_ids.insert(object.id);
        }

		collision_objects.push_back(object);
    }

	moveit::planning_interface::PlanningSceneInterface psi;
	std::vector<std::string> missing_ids;
	std::set_symmetric_difference(	current_object_ids.begin(), 	current_object_ids.end(),
									previous_object_ids_.begin(), 	previous_object_ids_.end(),
									std::back_inserter(missing_ids));
	psi.removeCollisionObjects(missing_ids);

    if (!psi.applyCollisionObjects(collision_objects)) {
		RCLCPP_ERROR(LOGGER, "Failed to apply collision objects");
	}

	previous_object_ids_ = current_object_ids;
}

void SprpiMainTaskNode::mirrorDetectionsCallback(const yolo_msgs::msg::DetectionArray::SharedPtr msg)
{   
	is_diseased_ = true;
	for (const auto &detection : msg->detections) {
		auto class_name = detection.class_name;

		if (class_name == "diseased") {
			is_diseased_ = true;
		} 
    }

	if (arm_states_.is_infront_of_mirror) {
        RCLCPP_INFO(
            LOGGER, "Creating and executing place task on: %s after determining it was %s", 
            (*previous_object_ids_.begin()).c_str(), 
            is_diseased_ ? "diseased" : "NOT diseased");
        arm_states_.current_task = createPlaceTask(is_diseased_, arm_states_.picked_object);
		doTask();
        arm_states_.is_picking = false;
        arm_states_.is_infront_of_mirror = false;
        previous_object_ids_.erase(arm_states_.picked_object);
        arm_states_.picked_object = "";
	}
}

void SprpiMainTaskNode::loopCallback()
{
    RCLCPP_INFO_ONCE(LOGGER, "Starting loop...");
	if (!arm_states_.is_picking && !previous_object_ids_.empty()) {
        RCLCPP_INFO(LOGGER, "Creating and executing pick task on: %s", (*previous_object_ids_.begin()).c_str());
        arm_states_.is_picking = true;
        arm_states_.picked_object = *previous_object_ids_.begin();
        arm_states_.current_task = createPickTask(*previous_object_ids_.begin());
		doTask();
        sleep(10);
	} 
}

mtc::Task SprpiMainTaskNode::createPickTask(const std::string object_id)
{
    initPlanners();
	
    mtc::Task task;
    task.stages()->setName("pick_task");
    task.loadRobotModel(node_);
  
    // Set task properties
    task.setProperty("group", arm_group_name_);
    task.setProperty("eef", hand_group_name_);
    task.setProperty("ik_frame", hand_frame_);
// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  	mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    auto stage_default_position = std::make_unique<mtc::stages::MoveTo>("move to default", interpolation_planner_);
    stage_default_position->setGroup(arm_group_name_);
    stage_default_position->setGoal("Default");
    task.add(std::move(stage_default_position));

    auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner_);
    stage_open_hand->setGroup(hand_group_name_);
    stage_open_hand->setGoal("open");
    task.add(std::move(stage_open_hand));

    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
        "move to pick",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner_ } });
    stage_move_to_pick->setTimeout(5.0);
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_pick));

    mtc::Stage* attach_object_stage =
    nullptr;  // Forward attach_object_stage to place pose generator

    {
        auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
        task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
        grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                              { "eef", "group", "ik_frame" });
        {
          auto stage =
              std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner_);
          stage->properties().set("marker_ns", "approach_object");
          stage->properties().set("link", hand_frame_);
          stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
          stage->setMinMaxDistance(0.085, 0.15);
        
          // Set hand forward direction
          geometry_msgs::msg::Vector3Stamped vec;
          vec.header.frame_id = hand_frame_;
          vec.vector.z = 1.0;
          stage->setDirection(vec);
          grasp->insert(std::move(stage));
        }
    
        {
          // Sample grasp pose
          auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
          stage->properties().configureInitFrom(mtc::Stage::PARENT);
          stage->properties().set("marker_ns", "grasp_pose");
          stage->setPreGraspPose("open");
          stage->setObject(object_id);
          stage->setAngleDelta(M_PI / ik_params_.angle_delta);
          stage->setMonitoredStage(current_state_ptr);  // Hook into current state
    
          Eigen::Isometry3d grasp_frame_transform;
          Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                                Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
          grasp_frame_transform.linear() = q.matrix();
          grasp_frame_transform.translation().z() = 0.1;
    
            // Compute IK
          auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
          wrapper->setMaxIKSolutions(ik_params_.max_ik_solutions);
          wrapper->setMinSolutionDistance(ik_params_.min_solution_distance);
          wrapper->setIKFrame(grasp_frame_transform, hand_frame_);
          wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
          wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
          grasp->insert(std::move(wrapper));
        }
    
        {
          auto stage =
              std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
          stage->allowCollisions(object_id,
                                task.getRobotModel()
                                    ->getJointModelGroup(hand_group_name_)
                                    ->getLinkModelNamesWithCollisionGeometry(),
                                true);
          grasp->insert(std::move(stage));
        }
    
        {
          auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner_);
          stage->setGroup(hand_group_name_);
          stage->setGoal("close");
          grasp->insert(std::move(stage));
        }
    
        {
          auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
          stage->attachObject(object_id, hand_frame_);
          attach_object_stage = stage.get();
          grasp->insert(std::move(stage));
        }
    
        {
          auto stage =
              std::make_unique<mtc::stages::MoveRelative>("retreat from object", cartesian_planner_);
          stage->properties().set("marker_ns", "retreat_from_object");
          stage->properties().set("link", hand_frame_);
          stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
          stage->setMinMaxDistance(0.085, 0.15);
        
          // Set hand forward direction
          geometry_msgs::msg::Vector3Stamped vec;
          vec.header.frame_id = hand_frame_;
          vec.vector.z = -1.0;
          stage->setDirection(vec);
          grasp->insert(std::move(stage));
        }
    
        {
          auto stage =
              std::make_unique<mtc::stages::MoveTo>("bring object infront of mirror", sampling_planner_);
          stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
          stage->setGroup(arm_group_name_);
          stage->setGoal("infront_of_mirror");
          stage->properties().set("marker_ns", "infront_of_mirror");
    
          grasp->insert(std::move(stage));
        }
    
        task.add(std::move(grasp));
      }
    
	arm_states_.is_infront_of_mirror = true;
    return task;
}

mtc::Task SprpiMainTaskNode::createPlaceTask(bool is_diseased, const std::string object_id)
{
    initPlanners();
	std::string basket = (is_diseased) ? "pre_drop_diseased" : "pre_drop_healthy";
	mtc::Task task;
    task.stages()->setName("place_task");
    task.loadRobotModel(node_);
  
    // Set task properties
    task.setProperty("group", arm_group_name_);
    task.setProperty("eef", hand_group_name_);
    task.setProperty("ik_frame", hand_frame_);
// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  	mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    {
		auto place = std::make_unique<mtc::SerialContainer>("place into basket");
		task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
        place->properties().configureInitFrom(mtc::Stage::PARENT,
                                              { "eef", "group", "ik_frame" });
		{
			auto stage = std::make_unique<mtc::stages::MoveTo>("raise from mirror", interpolation_planner_);
			stage->setGroup(arm_group_name_);
			stage->setGoal("raise_from_mirror");
			place->insert(std::move(stage));
		}

		{
			auto stage = std::make_unique<mtc::stages::MoveTo>("rotate to baskets", interpolation_planner_);
			stage->setGroup(arm_group_name_);
			stage->setGoal("rotate_to_basket");
			place->insert(std::move(stage));
		}

		{
			auto stage = std::make_unique<mtc::stages::MoveTo>("move to basket", interpolation_planner_);
			stage->setGroup(arm_group_name_);
			stage->setGoal(basket);
			place->insert(std::move(stage));
		}

		{
			auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner_);
			stage->setGroup(hand_group_name_);
			stage->setGoal("open");
			place->insert(std::move(stage));
		}

		{
			auto stage =
				std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
			stage->allowCollisions(object_id,
								  task.getRobotModel()
									  ->getJointModelGroup(hand_group_name_)
									  ->getLinkModelNamesWithCollisionGeometry(),
								  false);
			place->insert(std::move(stage));
		}

		{
			auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
			stage->detachObject(object_id, hand_frame_);
			place->insert(std::move(stage));
		}

		{
			auto stage = std::make_unique<mtc::stages::MoveTo>("move to default position", interpolation_planner_);
			stage->setGroup(arm_group_name_);
			stage->setGoal("Default");
			place->insert(std::move(stage));
		}
	}

	return task;
}

void SprpiMainTaskNode::doTask()
{
	try
	{
		arm_states_.current_task.init();
	}
	catch (mtc::InitStageException& e)
	{
		RCLCPP_ERROR_STREAM(LOGGER, e);
		return;
	}

	if (!arm_states_.current_task.plan(5))
	{
		RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
		return;
	}
	arm_states_.current_task.introspection().publishSolution(*arm_states_.current_task.solutions().front());

	auto result = arm_states_.current_task.execute(*arm_states_.current_task.solutions().front());
	if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
	{
		RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed with error code: " << result.val);
		RCLCPP_ERROR_STREAM(LOGGER, ", error message: " << result.message);
		RCLCPP_ERROR_STREAM(LOGGER, ", error source: " << result.source);
		return;
	}
	return;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto main_task_node = std::make_shared<SprpiMainTaskNode>(options);
  main_task_node->initPlanners();
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &main_task_node]() {
    executor.add_node(main_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(main_task_node->getNodeBaseInterface());
  });

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}

