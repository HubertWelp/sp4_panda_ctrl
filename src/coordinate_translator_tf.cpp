#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Vector3.h>
#include <iostream>

static const double MM_TO_M = 1.0 / 1000.0;

class SP4CoordinateTranslator
{
public:
  SP4CoordinateTranslator() : tf_listener_(tf_buffer_)
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    subscribed_topic_name_ = "/robot/pose";
    published_topic_name_  = "/sweet_pose_translated";

    // Ziel-Frame für den Panda-Controller
    pnh.param<std::string>("output_frame", output_frame_, std::string("panda_link0"));

    // Ursprung (0,0) der Bildverarbeitung im Roboter-Frame (panda_link0).
    // Wird aus der festen Foto-Pose hergeleitet:
    // TCP ~ (0.405, ...), Kamera +0.05 m in +X, Bildhöhe 0.22 m -> origin_x ~ 0.57
    // Linsenoffset +0.035 m in +Y, Bildbreite 0.30 m -> origin_y ~ 0.185
    pnh.param<std::string>("world_frame",world_frame_,"panda_link0");
    pnh.param<std::string>("camera_frame",camera_frame_,"camera_color_optical_frame");
    pnh.param<double>("table_z",table_z_,0.0);

    // camera intrinsics (taken from /camera_info)
    pnh.param("fx", fx_, 554.38);
    pnh.param("fy", fy_, 554.38);
    pnh.param("cx", cx_, 320.5);
    pnh.param("fx", cy_, 240.5);

    sub_ = nh.subscribe(subscribed_topic_name_, 10, &SP4CoordinateTranslator::poseCallback, this);
    pub_ = nh.advertise<geometry_msgs::PoseStamped>(published_topic_name_, 10);

    ROS_INFO_STREAM("[coordinate_translator] sub=" << subscribed_topic_name_
                    << " pub=" << published_topic_name_
                    << " frame=" << output_frame_);
  }

private:

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    // currently SP4 sends coordinates from top-left of the image in mm
    // The scaling between pixels (u,v) and distance is given by (values from konfig.ini):
    // - dx= 300/640*u
    // - dy= -220/480*v

    double u,v;
    u=640/300*msg->pose.position.x;
    v=-480/220*msg->pose.position.y; // SP4 changes y-direction

    // ray (object) direction in camera frame
    // I am not sure whether this is correct, since in RViz the axis showing down from
    // the camera to the table is the x-axis ([z,y,x]).
    tf2::Vector3 d_c((u-cx_)/fx_,(v-cy_)/fy_,1.0);
//    tf2::Vector3 d_c(1.0,(u-cx_)/fx_,(v-cy_)/fy_);
    d_c.normalize();

    try{
      // get transform world <- camera
      geometry_msgs::TransformStamped T_w_c
        = tf_buffer_.lookupTransform(world_frame_,camera_frame_,ros::Time(0),ros::Duration(0.1));

      // ray origin in world frame is camera origin transformed
      tf2::Vector3 o_w( T_w_c.transform.translation.x,
                        T_w_c.transform.translation.y,
                        T_w_c.transform.translation.z
      );

      // rotate ray direction into world frame
      tf2::Quaternion q;
      tf2::fromMsg(T_w_c.transform.rotation,q);
      tf2::Vector3 d_w = tf2::quatRotate(q,d_c);

      // compute intersection of ray with table plane (z=table_z_)
      if (std::abs(d_w.z())<1e-9)
      {
        ROS_WARN_STREAM_THROTTLE(1.0,"Ray is parallel to table plane, cannot compute intersection.");
        return;
      }
      double t = (table_z_ - o_w.z())/d_w.z();
      if(t<=0)
      {
        ROS_WARN_STREAM_THROTTLE(1.0,"Intersection is behind the camera, invalid measurement.");
        return;
      }
      tf2::Vector3 p_w = o_w + t*d_w;

      geometry_msgs::PoseStamped out;
      out.header.stamp = ros::Time::now();
      out.header.frame_id = output_frame_;

      out.pose.position.x = p_w.x();
      out.pose.position.y = p_w.y();
      out.pose.position.z = p_w.z();

      out.pose.orientation.x = msg->pose.orientation.x;
      out.pose.orientation.y = msg->pose.orientation.y;
      out.pose.orientation.z = msg->pose.orientation.z;
      out.pose.orientation.w = msg->pose.orientation.w;

      pub_.publish(out);

    }catch(const tf2::TransformException &ex){
      ROS_WARN_STREAM_THROTTLE(1.0,"Could not get transform from " << camera_frame_ << " to " << world_frame_ << ": " << ex.what());
      return;
    }


  }

  // class member variables
  ros::Subscriber sub_;
  ros::Publisher  pub_;
  std::string subscribed_topic_name_;
  std::string published_topic_name_;
  std::string output_frame_;

  std::string world_frame_;
  std::string camera_frame_;
  double table_z_;

  double fx_,fy_,cx_,cy_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv)
{
  std::cout << "Coordinate Translator TF" << std::endl;
  ros::init(argc, argv, "coordinate_translator_tf");
  SP4CoordinateTranslator translator;
  ros::spin();
  return 0;
}
