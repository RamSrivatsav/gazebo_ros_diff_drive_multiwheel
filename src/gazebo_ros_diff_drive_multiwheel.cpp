/*
    Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
    All rights reserved.
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.
    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * \file  gazebo_ros_diff_drive.cpp
 *
 * \brief A differential drive plugin for gazebo. Based on the diffdrive plugin
 * developed for the erratic robot (see copyright notice above). The original
 * plugin can be found in the ROS package gazebo_erratic_plugins.
 *
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * $ Id: 06/21/2013 11:23:40 AM piyushk $
 */


/*
 *
 * The support of acceleration limit was added by
 * \author   George Todoran <todorangrg@gmail.com>
 * \author   Markus Bader <markus.bader@tuwien.ac.at>
 * \date 22th of May 2014
 */


/*
 * \file  gazebo_ros_diff_drive_multiwheel.cpp
 *
 * \brief A Multi wheel differential drive plugin for gazebo. Based on the diffdrive plugin
 * developed for the erratic robot (see copyright notice above). The original
 * plugin can be found in the ROS package gazebo_erratic_plugins.
 *
 * \author  Ram Srivatsav (vatsav.ben@gmail.com)
 *
 * $ Id: 12/16/2020 3:29 PM Ram $
 */

#include <algorithm>
#include <assert.h>

#include <gazebo_ros_diff_drive_multiwheel/gazebo_ros_diff_drive_multiwheel.h>

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <gazebo/gazebo_config.h>

namespace gazebo
{

enum {
    RIGHT,
    LEFT,
};

GazeboRosDiffDriveMW::GazeboRosDiffDriveMW() {}

// Destructor
GazeboRosDiffDriveMW::~GazeboRosDiffDriveMW() 
{
	FiniChild();
}

// Load the controller
void GazeboRosDiffDriveMW::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{

    this->parent = _parent;
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "DiffDriveMW" ) );
    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->isInitialized();

    gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopic", "cmd_vel" );
    gazebo_ros_->getParameter<std::string> ( odometry_topic_, "odometryTopic", "odom" );
    gazebo_ros_->getParameter<std::string> ( odometry_frame_, "odometryFrame", "odom" );
    gazebo_ros_->getParameter<std::string> ( robot_base_frame_, "robotBaseFrame", "base_footprint" );
    gazebo_ros_->getParameterBoolean ( publishWheelTF_, "publishWheelTF", false );
    gazebo_ros_->getParameterBoolean ( publishOdomTF_, "publishOdomTF", true);
    gazebo_ros_->getParameterBoolean ( publishWheelJointState_, "publishWheelJointState", false );
    gazebo_ros_->getParameterBoolean ( legacy_mode_, "legacyMode", true );

    if (!_sdf->HasElement("legacyMode"))
    {
      ROS_ERROR_NAMED("diff_drive_MW", "GazeboRosDiffDriveMW Plugin missing <legacyMode>, defaults to true\n"
	       "This setting assumes you have a old package, where the right and left wheel are changed to fix a former code issue\n"
	       "To get rid of this error just set <legacyMode> to false if you just created a new package.\n"
	       "To fix an old package you have to exchange left wheel by the right wheel.\n"
	       "If you do not want to fix this issue in an old package or your z axis points down instead of the ROS standard defined in REP 103\n"
	       "just set <legacyMode> to true.\n"
      );
    }

    gazebo_ros_->getParameter<double> ( wheel_separation_, "wheelSeparation", 0.34 );
    gazebo_ros_->getParameter<double> ( wheel_diameter_, "wheelDiameter", 0.15 );
    gazebo_ros_->getParameter<double> ( wheel_accel, "wheelAcceleration", 0.0 );
    gazebo_ros_->getParameter<double> ( wheel_torque, "wheelTorque", 5.0 );
    gazebo_ros_->getParameter<double> ( update_rate_, "updateRate", 100.0 );
    std::map<std::string, OdomSource> odomOptions;
    odomOptions["encoder"] = ENCODER;
    odomOptions["world"] = WORLD;
    gazebo_ros_->getParameter<OdomSource> ( odom_source_, "odometrySource", odomOptions, WORLD );


    if (!_sdf->HasElement("leftJoints")) {
      gzthrow("Have to specify space separated left side joint names via <leftJoints> tag!");
    } else {
      std::string joint_string = _sdf->GetElement("leftJoints")->Get<std::string>();
      boost::split( joint_names_[LEFT], joint_string, boost::is_any_of(" ") );
    }

    if (!_sdf->HasElement("rightJoints")) {
      gzthrow("Have to specify space separated right side joint names via <rightJoints> tag!");
    } else {
      std::string joint_string = _sdf->GetElement("rightJoints")->Get<std::string>();
      boost::split( joint_names_[RIGHT], joint_string, boost::is_any_of(" ") );
    }

    this->publish_tf_ = true;
    if (!_sdf->HasElement("publishTf")) {
      ROS_WARN_NAMED("diff_drive_MW", "GazeboRosDiffDriveMW Plugin (ns = %s) missing <publishTf>, defaults to %d",
          this->robot_namespace_.c_str(), this->publish_tf_);
    } else {
      this->publish_tf_ = _sdf->GetElement("publishTf")->Get<bool>();
    }

    // Initialize update rate stuff
    if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_;
    else this->update_period_ = 0.0;
#if GAZEBO_MAJOR_VERSION >= 8
    last_update_time_ = parent->GetWorld()->SimTime();
#else
    last_update_time_ = parent->GetWorld()->GetSimTime();
#endif

    // Initialize velocity stuff
    wheel_speed_[RIGHT] = 0;
    wheel_speed_[LEFT] = 0;

    // Initialize velocity support stuff
    wheel_speed_instr_[RIGHT] = 0;
    wheel_speed_instr_[LEFT] = 0;

    x_ = 0;
    rot_ = 0;
    alive_ = true;

    for (size_t side = 0; side < 2; ++side){
      for (size_t i = 0; i < joint_names_[side].size(); ++i){
        joints_[side].push_back(this->parent->GetJoint(joint_names_[side][i]));
        if (!joints_[side][i]){
          char error[200];
          snprintf(error, 200,
                   "GazeboRosDiffDriveMultiWheel Plugin (ns = %s) couldn't get hinge joint named \"%s\"",
                   this->robot_namespace_.c_str(), joint_names_[side][i].c_str());
          gzthrow(error);
        }
        joints_[side][i]->SetParam ( "fmax", 0, wheel_torque );
      }
    }


    if (this->publishWheelJointState_)
    {
        joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>("joint_states", 1000);
        ROS_INFO_NAMED("diff_drive_MW", "%s: Advertise joint_states", gazebo_ros_->info());
    }

    transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ROS_INFO_NAMED("diff_drive_MW", "%s: Try to subscribe to %s", gazebo_ros_->info(), command_topic_.c_str());

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
                boost::bind(&GazeboRosDiffDriveMW::cmdVelCallback, this, _1),
                ros::VoidPtr(), &queue_);

    cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
    ROS_INFO_NAMED("diff_drive_MW", "%s: Subscribe to %s", gazebo_ros_->info(), command_topic_.c_str());

    if (this->publish_tf_)
    {
      odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
      ROS_INFO_NAMED("diff_drive_MW", "%s: Advertise odom on %s ", gazebo_ros_->info(), odometry_topic_.c_str());
    }

    // start custom queue for diff drive
    this->callback_queue_thread_ =
        boost::thread ( boost::bind ( &GazeboRosDiffDriveMW::QueueThread, this ) );

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
        event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosDiffDriveMW::UpdateChild, this ) );

}

void GazeboRosDiffDriveMW::Reset()
{
#if GAZEBO_MAJOR_VERSION >= 8
  last_update_time_ = parent->GetWorld()->SimTime();
#else
  last_update_time_ = parent->GetWorld()->GetSimTime();
#endif
  pose_encoder_.x = 0;
  pose_encoder_.y = 0;
  pose_encoder_.theta = 0;
  x_ = 0;
  rot_ = 0;

  for (size_t side = 0; side < 2; ++side){
    for (size_t i = 0; i < joint_names_[side].size(); ++i){
      joints_[side][i]->SetParam ( "fmax", 0, wheel_torque );
    }
  }
}

void GazeboRosDiffDriveMW::publishWheelJointState() // need to change this.
{
    ros::Time current_time = ros::Time::now();

    joint_state_.header.stamp = current_time;
    joint_state_.name.resize ( 2*joint_names_[LEFT].size() );
    joint_state_.position.resize ( 2*joint_names_[LEFT].size() );
    for (size_t side = 0; side < 2; ++side){
      for (size_t i = 0; i < joint_names_[side].size(); ++i){
        physics::JointPtr joint = joints_[side][i];
#if GAZEBO_MAJOR_VERSION >= 8
        double position = joint->Position ( 0 );
#else
        double position = joint->GetAngle ( 0 ).Radian();
#endif
        joint_state_.name[i+joint_names_[LEFT].size()*side] = joint->GetName();
        joint_state_.position[i+joint_names_[LEFT].size()*side] = position;
      }
    }
    joint_state_publisher_.publish ( joint_state_ );
}

void GazeboRosDiffDriveMW::publishWheelTF()
{
    ros::Time current_time = ros::Time::now();

    for (size_t side = 0; side < 2; ++side){
      for (size_t i = 0; i < joint_names_[side].size(); ++i){

          std::string wheel_frame = gazebo_ros_->resolveTF(joints_[side][i]->GetChild()->GetName ());
          std::string wheel_parent_frame = gazebo_ros_->resolveTF(joints_[side][i]->GetParent()->GetName ());

#if GAZEBO_MAJOR_VERSION >= 8
          ignition::math::Pose3d poseWheel = joints_[side][i]->GetChild()->RelativePose();
#else
          ignition::math::Pose3d poseWheel = joints_[side][i]->GetChild()->GetRelativePose().Ign();
#endif

          tf::Quaternion qt ( poseWheel.Rot().X(), poseWheel.Rot().Y(), poseWheel.Rot().Z(), poseWheel.Rot().W() );
          tf::Vector3 vt ( poseWheel.Pos().X(), poseWheel.Pos().Y(), poseWheel.Pos().Z() );

          tf::Transform tfWheel ( qt, vt );
          transform_broadcaster_->sendTransform (
              tf::StampedTransform ( tfWheel, current_time, wheel_parent_frame, wheel_frame ) );
      }
    }
}

// Update the controller
void GazeboRosDiffDriveMW::UpdateChild()
{

    /* force reset SetParam("fmax") since Joint::Reset reset MaxForce to zero at
       https://bitbucket.org/osrf/gazebo/src/8091da8b3c529a362f39b042095e12c94656a5d1/gazebo/physics/Joint.cc?at=gazebo2_2.2.5#cl-331
       (this has been solved in https://bitbucket.org/osrf/gazebo/diff/gazebo/physics/Joint.cc?diff2=b64ff1b7b6ff&at=issue_964 )
       and Joint::Reset is called after ModelPlugin::Reset, so we need to set maxForce to wheel_torque other than GazeboRosDiffDriveMW::Reset
       (this seems to be solved in https://bitbucket.org/osrf/gazebo/commits/ec8801d8683160eccae22c74bf865d59fac81f1e)
    */
    for (size_t side = 0; side < 2; ++side){
      for (size_t i = 0; i < joint_names_[side].size(); ++i){
        joints_[side][i]->SetVelocity(0, wheel_speed_[side] / (0.5 * wheel_diameter_));
      }
    }


    if ( odom_source_ == ENCODER ) UpdateOdometryEncoder();
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->SimTime();
#else
    common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();

    if ( seconds_since_last_update > update_period_ ) {
        if (this->publish_tf_) publishOdometry ( seconds_since_last_update );
        if ( publishWheelTF_ ) publishWheelTF();
        if ( publishWheelJointState_ ) publishWheelJointState();

        // Update robot in case new velocities have been requested
        getWheelVelocities();

        double current_speed[2];

        current_speed[LEFT] = joints_[LEFT][0]->GetVelocity ( 0 )   * ( wheel_diameter_ / 2.0 );
        current_speed[RIGHT] = joints_[RIGHT][0]->GetVelocity ( 0 ) * ( wheel_diameter_ / 2.0 );


        for (size_t side = 0; side < 2; ++side){
          for (size_t i = 0; i < joint_names_[side].size(); ++i){
            if ( wheel_accel == 0 ||
                    ( fabs ( wheel_speed_[LEFT] - current_speed[LEFT] ) < 0.01 ) ||
                    ( fabs ( wheel_speed_[RIGHT] - current_speed[RIGHT] ) < 0.01 ) ) {
                joints_[side][i]->SetParam ( "vel", 0, wheel_speed_[side]/ ( wheel_diameter_ / 2.0 ) );
            } else {
                if ( wheel_speed_[side]>current_speed[side] )
                    wheel_speed_instr_[side]+=fmin ( wheel_speed_[side]-current_speed[side], wheel_accel * seconds_since_last_update );
                else
                    wheel_speed_instr_[side]+=fmax ( wheel_speed_[side]-current_speed[side], -wheel_accel * seconds_since_last_update );
                joints_[side][i]->SetParam ( "vel", 0, wheel_speed_instr_[side] / ( wheel_diameter_ / 2.0 ) );
            }
          }
        }
        last_update_time_+= common::Time ( update_period_ );
    }
}

// Finalize the controller
void GazeboRosDiffDriveMW::FiniChild()
{
    alive_ = false;
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}

void GazeboRosDiffDriveMW::getWheelVelocities()
{
    boost::mutex::scoped_lock scoped_lock ( lock );

    double vr = x_;
    double va = rot_;

    if(legacy_mode_)
    {
      wheel_speed_[LEFT] = vr + va * wheel_separation_ / 2.0;
      wheel_speed_[RIGHT] = vr - va * wheel_separation_ / 2.0;
    }
    else
    {
      wheel_speed_[LEFT] = vr - va * wheel_separation_ / 2.0;
      wheel_speed_[RIGHT] = vr + va * wheel_separation_ / 2.0;
    }
}

void GazeboRosDiffDriveMW::cmdVelCallback ( const geometry_msgs::Twist::ConstPtr& cmd_msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    x_ = cmd_msg->linear.x;
    rot_ = cmd_msg->angular.z;
}

void GazeboRosDiffDriveMW::QueueThread()
{
    static const double timeout = 0.01;

    while ( alive_ && gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

void GazeboRosDiffDriveMW::UpdateOdometryEncoder()
{
    double vl = joints_[LEFT][0]->GetVelocity ( 0 );
    double vr = joints_[RIGHT][0]->GetVelocity ( 0 );
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->SimTime();
#else
    common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
    double seconds_since_last_update = ( current_time - last_odom_update_ ).Double();
    last_odom_update_ = current_time;

    double b = wheel_separation_;

    // Book: Sigwart 2011 Autonompus Mobile Robots page:337
    double sl = vl * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    double sr = vr * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    double ssum = sl + sr;

    double sdiff;
    if(legacy_mode_)
    {
      sdiff = sl - sr;
    }
    else
    {

      sdiff = sr - sl;
    }

    double dx = ( ssum ) /2.0 * cos ( pose_encoder_.theta + ( sdiff ) / ( 2.0*b ) );
    double dy = ( ssum ) /2.0 * sin ( pose_encoder_.theta + ( sdiff ) / ( 2.0*b ) );
    double dtheta = ( sdiff ) /b;

    pose_encoder_.x += dx;
    pose_encoder_.y += dy;
    pose_encoder_.theta += dtheta;

    double w = dtheta/seconds_since_last_update;
    double v = sqrt ( dx*dx+dy*dy ) /seconds_since_last_update;

    tf::Quaternion qt;
    tf::Vector3 vt;
    qt.setRPY ( 0,0,pose_encoder_.theta );
    vt = tf::Vector3 ( pose_encoder_.x, pose_encoder_.y, 0 );

    odom_.pose.pose.position.x = vt.x();
    odom_.pose.pose.position.y = vt.y();
    odom_.pose.pose.position.z = vt.z();

    odom_.pose.pose.orientation.x = qt.x();
    odom_.pose.pose.orientation.y = qt.y();
    odom_.pose.pose.orientation.z = qt.z();
    odom_.pose.pose.orientation.w = qt.w();

    odom_.twist.twist.angular.z = w;
    odom_.twist.twist.linear.x = dx/seconds_since_last_update;
    odom_.twist.twist.linear.y = dy/seconds_since_last_update;
}

void GazeboRosDiffDriveMW::publishOdometry ( double step_time )
{

    ros::Time current_time = ros::Time::now();
    std::string odom_frame = gazebo_ros_->resolveTF ( odometry_frame_ );
    std::string base_footprint_frame = gazebo_ros_->resolveTF ( robot_base_frame_ );

    tf::Quaternion qt;
    tf::Vector3 vt;

    if ( odom_source_ == ENCODER ) {
        // getting data form encoder integration
        qt = tf::Quaternion ( odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w );
        vt = tf::Vector3 ( odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z );

    }
    if ( odom_source_ == WORLD ) {
        // getting data from gazebo world
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d pose = parent->WorldPose();
#else
        ignition::math::Pose3d pose = parent->GetWorldPose().Ign();
#endif
        qt = tf::Quaternion ( pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W() );
        vt = tf::Vector3 ( pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z() );

        odom_.pose.pose.position.x = vt.x();
        odom_.pose.pose.position.y = vt.y();
        odom_.pose.pose.position.z = vt.z();

        odom_.pose.pose.orientation.x = qt.x();
        odom_.pose.pose.orientation.y = qt.y();
        odom_.pose.pose.orientation.z = qt.z();
        odom_.pose.pose.orientation.w = qt.w();

        // get velocity in /odom frame
        ignition::math::Vector3d linear;
#if GAZEBO_MAJOR_VERSION >= 8
        linear = parent->WorldLinearVel();
        odom_.twist.twist.angular.z = parent->WorldAngularVel().Z();
#else
        linear = parent->GetWorldLinearVel().Ign();
        odom_.twist.twist.angular.z = parent->GetWorldAngularVel().Ign().Z();
#endif

        // convert velocity to child_frame_id (aka base_footprint)
        float yaw = pose.Rot().Yaw();
        odom_.twist.twist.linear.x = cosf ( yaw ) * linear.X() + sinf ( yaw ) * linear.Y();
        odom_.twist.twist.linear.y = cosf ( yaw ) * linear.Y() - sinf ( yaw ) * linear.X();
    }

    if (publishOdomTF_ == true){
        tf::Transform base_footprint_to_odom ( qt, vt );
        transform_broadcaster_->sendTransform (
            tf::StampedTransform ( base_footprint_to_odom, current_time,
                                   odom_frame, base_footprint_frame ) );
    }


    // set covariance
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;


    // set header
    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    odometry_publisher_.publish ( odom_ );
}

GZ_REGISTER_MODEL_PLUGIN ( GazeboRosDiffDriveMW )
}