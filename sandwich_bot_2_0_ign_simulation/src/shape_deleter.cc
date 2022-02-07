#include <iostream>
#include <ignition/transport.hh>
#include <ignition/msgs.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/math/Vector3.hh>
#include "rclcpp/rclcpp.hpp"
#include "sandwich_bot_2_0_interfaces/srv/delete_shape.hpp"
using namespace ignition;
using namespace gazebo;

std::string world_name;
long unsigned int sandwich_bot;

const float max_delete_distance = 1.0;

std::string getWorldName()
{
  // Create a transport node.
  ignition::transport::Node node;

  bool result{false};
  const unsigned int timeout{5000};
  const std::string service{"/gazebo/worlds"};

  // Request and block
  ignition::msgs::StringMsg_V res;

  if (!node.Request(service, timeout, res, result))
  {
    std::cerr << std::endl << "Service call to [" << service << "] timed out"
              << std::endl;
    return "";
  }

  if (!result)
  {
    std::cerr << std::endl << "Service call to [" << service << "] failed"
              << std::endl;
    return "";
  }

  return res.data().Get(0);
}

bool populateECM(ignition::gazebo::EntityComponentManager &_ecm, const std::string world)
{
  // Create a transport node.
  ignition::transport::Node node;
  bool result{false};
  const unsigned int timeout{5000};
  const std::string service{"/world/" + world + "/state"};


  // Request and block
  ignition::msgs::SerializedStepMap res;

  if (!node.Request(service, timeout, res, result))
  {
    std::cerr << std::endl << "Service call to [" << service << "] timed out"
              << std::endl;
    return false;
  }

  if (!result)
  {
    std::cerr << std::endl << "Service call to [" << service << "] failed"
              << std::endl;
    return false;
  }

  // Instantiate an ECM and populate with data from message
  _ecm.SetState(res.state());
  return true;
}

std::vector<long unsigned int> get_shape_entities_list(const ignition::gazebo::EntityComponentManager &ecm, std::string shape_type){
    std::vector<long unsigned int> shapes;
    ecm.EachNoCache<components::Link,
           components::ParentEntity,
           components::Name>(
        [&](const Entity &_entity,
            const components::Link *_link,
            const components::ParentEntity *_parent,
            const components::Name *_name) -> bool
        {
            if(_name->Data() == shape_type)
                shapes.push_back(_parent->Data());
            return true;
        });

        return shapes;
}

void exception_handler(const bool &executed , const bool &result, const std::string &service){
    if (executed)
    {
        if (!result)
            throw std::runtime_error( "Service call to [" + service + "] failed.");
    }
    else
        throw std::runtime_error( "Service call to [" + service + "] timed out.");
}

void delete_box(ignition::gazebo::EntityComponentManager &ecm, const std::string world_name, const long unsigned int box_id){
    ignition::transport::Node node;
    ignition::msgs::Entity req;
    ignition::msgs::Boolean rep;
    bool result;
    unsigned int timeout = 2000;
    std::string service = "/world/" + world_name + "/remove";
    const auto box_name = ecm.Component<ignition::gazebo::components::Name>(box_id);
    req.set_name(box_name->Data());
    req.set_id(box_id);
    bool executed = node.Request(service, req,timeout,rep, result);
    exception_handler(executed, result, service);
}

ignition::math::Pose3d get_bot_position(const ignition::gazebo::EntityComponentManager &ecm, const long unsigned int sandwich_bot){
    
    return ecm.Component<ignition::gazebo::components::Pose>(sandwich_bot)->Data();
}

void delete_shape(const std::shared_ptr<sandwich_bot_2_0_interfaces::srv::DeleteShape::Request> request,
          std::shared_ptr<sandwich_bot_2_0_interfaces::srv::DeleteShape::Response>      response)
{
    ignition::gazebo::EntityComponentManager ecm;

    if (!populateECM(ecm, world_name))
    {
        throw std::runtime_error("populate ECM failed!");
    }

    auto bot_pose = get_bot_position(ecm, sandwich_bot);
    auto shape_list = get_shape_entities_list(ecm, request->shape_type);
    
    double min_dist = -1.0;
    std::size_t shape;
    for( int i=0;i<shape_list.size();i++){
        if(i==0){
            shape = shape_list[0];
            auto shape_position = ecm.Component<ignition::gazebo::components::Pose>(shape)->Data();
            min_dist = bot_pose.Pos().Distance(shape_position.Pos());
        }else{
            auto shape_position = ecm.Component<ignition::gazebo::components::Pose>(shape_list[i])->Data();
            double temp_min_dist = bot_pose.Pos().Distance(shape_position.Pos());
            if(temp_min_dist < min_dist){
                min_dist = temp_min_dist;
                shape = shape_list[i];
            }
        }
    }
    if(min_dist > 0.0 && min_dist < max_delete_distance){
        delete_box(ecm, getWorldName(), shape);
    }

    request->shape_type;
    response->result = true;
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("delete_shape");
    rclcpp::Service<sandwich_bot_2_0_interfaces::srv::DeleteShape>::SharedPtr service =
    node->create_service<sandwich_bot_2_0_interfaces::srv::DeleteShape>("delete_shape", &delete_shape);

    ignition::gazebo::EntityComponentManager ecm{};
    world_name = getWorldName();
    if (world_name.empty())
    {
      std::cerr << "Command failed when trying to get the world name of "
                << "the running simulation." << std::endl;
      return false;
    }
    
    if (!populateECM(ecm, world_name))
    {
        return false;
    }

    auto world = ecm.EntityByComponents(ignition::gazebo::components::World());
    if (kNullEntity == world)
    {
        throw std::runtime_error("No world found.");
    }
    sandwich_bot = ecm.EntityByComponents(ignition::gazebo::components::ParentEntity(world), ignition::gazebo::components::Model(), ignition::gazebo::components::Name("sandwich_bot"));

    rclcpp::spin(node);
    rclcpp::shutdown();
}
