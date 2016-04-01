#include <quadrotor_simulator/quad_model.h>

namespace gazebo
{
    void QuadModel::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model_ = _parent;
      this->world_ = this->model_->GetWorld();

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&QuadModel::OnUpdate, this, _1));

      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "quad_sim",
        ros::init_options::NoSigintHandler);
      }

      this->rosNode_.reset(new ros::NodeHandle("quad_sim"));
      
      //subscribe to forces topic
      ros::SubscribeOptions so = 
      ros::SubscribeOptions::create<geometry_msgs::Twist>(
                "/force",
                1,
                boost::bind(&QuadModel::ForceCb, this, _1),
                ros::VoidPtr(), &this->rosQueue_);
      this->rosSub_ = this->rosNode_->subscribe(so);

      this->rosQueueThread_ =
              std::thread(std::bind(&QuadModel::QueueThread, this));

      //IMU Publisher
      this->imu_pub_ = this->rosNode_->advertise<sensor_msgs::Imu>("/imu", 1, false);

      //initialize vectors for model
      this->gravity_ = this->world_->GetPhysicsEngine()->GetGravity();
      this->forces_ = math::Vector3(0,0,0);
      this->moments_ = math::Vector3(0,0,0);
    }

    void QuadModel::ForceCb(const geometry_msgs::Twist::ConstPtr& msg)
    {
        this->forces_ = math::Vector3(msg->linear.x, msg->linear.y, msg->linear.z);
        this->moments_ = math::Vector3(msg->angular.x, msg->angular.y, msg->angular.z);
    }

    void QuadModel::QueueThread()
    {
        static const double timeout = 0.01;
        while (this->rosNode_->ok())
        {
            this->rosQueue_.callAvailable(ros::WallDuration(timeout));
        }
    }

    // Called by the world update start event
    void QuadModel::OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      this->link_ = this->model_->GetLink();

      //publish forces and moments to model
      this->link_->AddRelativeForce(this->forces_);
      this->link_->AddRelativeTorque(this->moments_);

      //IMU publishing
      math::Quaternion att = (link_->GetWorldPose()).rot;
      math::Vector3 ang_vel = this->link_->GetRelativeAngularVel();
      math::Vector3 lin_acc = this->link_->GetRelativeLinearAccel() - att.RotateVectorReverse(gravity_);
      
      sensor_msgs::Imu imu;
      imu.header.stamp = ros::Time::now();

      imu.orientation.x = att.x;
      imu.orientation.y = att.y;
      imu.orientation.z = att.z;
      imu.orientation.w = att.w;

      imu.angular_velocity.x = ang_vel[0];
      imu.angular_velocity.y = ang_vel[1];
      imu.angular_velocity.z = ang_vel[2];

      imu.linear_acceleration.x = lin_acc[0];
      imu.linear_acceleration.y = lin_acc[1];
      imu.linear_acceleration.z = lin_acc[2];

      imu_pub_.publish(imu);
    
    }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(QuadModel)
}
