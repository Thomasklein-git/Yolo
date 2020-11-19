#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <iostream>

#include <sensor_msgs/PointCloud2.h>

#include <sensor_msgs/TimeReference.h>

using namespace sensor_msgs;
using namespace message_filters;


ros::Publisher pub_;


void callback_cloud(const sensor_msgs::PointCloud2ConstPtr& cloud, const sensor_msgs::TimeReferenceConstPtr& timer)
{

    
    sensor_msgs::Image image_;

    image_.header   = cloud -> header;
    image_.height   = cloud -> height;
    image_.width    = cloud -> width;
    image_.encoding = "32FC3";
    image_.is_bigendian = cloud -> is_bigendian;
    image_.step     = 8064;
    image_.data.resize (image_.step * image_.height);

    int offset = 0;
    int point_step = cloud -> point_step;
    
    for (size_t y = 0; y < cloud -> height; y++)
    {
        for (size_t x=0; x < cloud -> width; x++, offset += point_step)
        {
            //std::uint8_t * pixel = &(image_.data[y * image_.step + x*12]);
            std::uint8_t * pixel = &(image_.data[y * image_.step + x*4*3]);
            memcpy (pixel, &(cloud -> data[offset]), 3 * 4 * sizeof (std::uint8_t));
            //memcpy (pixel, &(cloud -> data[offset]), 12 * sizeof (std::uint8_t));
            }
    }
    pub_.publish(image_);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");
  ros::NodeHandle nh;
  
  pub_ = nh.advertise<sensor_msgs::Image> ("/Tracker/Pc2ToImage/Cloud", 1);
  
  message_filters::Subscriber<PointCloud2> cloud_sub(nh,"/zed2/zed_node/point_cloud/cloud_registered", 1);
  message_filters::Subscriber<TimeReference> timer_sub(nh,"/Tracker/Timer", 1);

  typedef sync_policies::ExactTime<PointCloud2, TimeReference> MySyncPolicy;
  // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(30), cloud_sub, timer_sub);
  sync.registerCallback(boost::bind(&callback_cloud, _1, _2));
  ros::spin();

  return 0;
}