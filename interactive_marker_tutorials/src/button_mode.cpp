/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <std_msgs/Bool.h>

#include <math.h>

using namespace visualization_msgs;

#define _MTM 0
#define _PSM 1 

// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
ros::Publisher publisher_mm_;
std_msgs::Bool mm_mode;
// %EndTag(vars)%


// %Tag(Box)%
Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

Marker makeText(InteractiveMarker &msg, std::string str)
{
    Marker marker;
    marker.type = Marker::TEXT_VIEW_FACING;
    marker.pose.orientation.x =  marker.pose.orientation.y  = marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.25;
    marker.color.r = 0.8;
    marker.color.g = 0.8;
    marker.color.b = 0.8;
    marker.color.a = 1.0;
    marker.text = str;
    return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}
// %EndTag(Box)%

// %Tag(frameCallback)%
void frameCallback(const ros::TimerEvent&)
{
  static uint32_t counter = 0;

  static tf::TransformBroadcaster br;

  tf::Transform t;

  ros::Time time = ros::Time::now();

  t.setOrigin(tf::Vector3(0.0, 0.0, sin(float(counter)/140.0) * 2.0));
  t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br.sendTransform(tf::StampedTransform(t, time, "base_link", "moving_frame"));

  t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  t.setRotation(tf::createQuaternionFromRPY(0.0, float(counter)/140.0, 0.0));
  br.sendTransform(tf::StampedTransform(t, time, "base_link", "rotating_frame"));

  counter++;
}
// %EndTag(frameCallback)%

// %Tag(processFeedback)%
void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int mode )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

//   switch ( feedback->event_type )
//   {
//     case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
//       ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
//       break;

//     case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
//       ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
//       break;

//     case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
//       ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
//       break;
//   }
  
  if (mode == _MTM) {
    ROS_INFO_STREAM("MTM measurement");
    mm_mode.data = true;
  } else {
    ROS_INFO_STREAM("PSM measurement");
    mm_mode.data = false;
  }

  publisher_mm_.publish(mm_mode);
  server->applyChanges();
}
// %EndTag(processFeedback)%
 
double rand( double min, double max )
{
  double t = (double)rand() / (double)RAND_MAX;
  return min + t*(max-min);
}

// %Tag(Button)%
void makeButtonMarker( const tf::Vector3& position, std::string str, int mode )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = str + "_button";
  // int_marker.description = "Button\n(Left Click)";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.name = str + "_button_control";

//   Marker marker = makeBox( int_marker );
  Marker marker = makeText( int_marker, str );

  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);

  if (mode == _PSM)
    server->setCallback(int_marker.name, boost::bind(&processFeedback, _1, mode));
  else
    server->setCallback(int_marker.name, boost::bind(&processFeedback, _1, mode));
}
// %EndTag(Button)%

// %Tag(main)%
int main(int argc, char** argv)
{
  ros::init(argc, argv, "button_mode");
  ros::NodeHandle n;
  publisher_mm_ = n.advertise<std_msgs::Bool>("/rvinci_measurement_MTM", 10);

  // create a timer to update the published transforms
  ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

  server.reset( new interactive_markers::InteractiveMarkerServer("button_mode","",false) );

  ros::Duration(0.1).sleep();

  tf::Vector3 position;
  position = tf::Vector3( -2, 0, 0); 
  makeButtonMarker( position, "PSM", _PSM);
  position = tf::Vector3( 2, 0, 0); 
  makeButtonMarker( position, "MTM", _MTM );

  server->applyChanges();
//   publisher_mm_.publish(mm_mode);

  ros::spin();

  server.reset();
}
// %EndTag(main)%
