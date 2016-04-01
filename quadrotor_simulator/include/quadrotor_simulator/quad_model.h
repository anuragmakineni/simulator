#include <stdio.h>
#include <memory>
#include <iostream>
#include <thread>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

namespace gazebo
{
    class QuadModel : public ModelPlugin
    {
        
        protected:
            void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
            void OnUpdate(const common::UpdateInfo & /*_info*/);
            void ForceCb(const geometry_msgs::Twist::ConstPtr& msg);
        private:
            void QueueThread();
            
            physics::WorldPtr world_;
            physics::ModelPtr model_;
            event::ConnectionPtr updateConnection_;
            physics::LinkPtr link_;

            std::unique_ptr<ros::NodeHandle> rosNode_;
            ros::Subscriber rosSub_;
            ros::CallbackQueue rosQueue_;
            std::thread rosQueueThread_;
            ros::Publisher imu_pub_;

            math::Vector3 gravity_;
            math::Vector3 forces_;
            math::Vector3 moments_;
            
    };
}
