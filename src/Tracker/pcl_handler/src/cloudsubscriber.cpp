// ROS core
#include <ros/ros.h>
//Image message
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
//pcl::toROSMsg
#include <pcl/io/pcd_io.h>
//conversions from PCL custom types
#include <pcl_conversions/pcl_conversions.h>
//stl stuff
#include <string>

#include <pcl/PCLPointField.h>
#include <pcl/PCLImage.h>
#include <pcl/PCLPointCloud2.h>

//https://github.com/PointCloudLibrary/pcl/blob/3b6faad3c1a851ef08b4dbdef47773f2787d3501/common/include/pcl/conversions.h

class PointCloudImage
{
public:
    void
    callback_cloud(const sensor_msgs::PointCloud2ConstPtr& cloud)
    {
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
        pub_.publish (image_);
    }
    PointCloudImage () : sub_topic_("/zed2/zed_node/point_cloud/cloud_registered"),pub_topic_("/Tracker/Pc2ToImage/Cloud")
    {
        sub_ = nh_.subscribe (sub_topic_, 1, &PointCloudImage::callback_cloud, this);
        pub_ = nh_.advertise<sensor_msgs::Image > (pub_topic_, 1);
    }
private:

private:
    ros::NodeHandle nh_;
    sensor_msgs::Image image_; //cache the image message
    std::string sub_topic_; //default input
    std::string pub_topic_; //default output
    ros::Subscriber sub_; //cloud subscriber
    ros::Publisher  pub_; //image message publisher
};

int 
main(int argc, char **argv)
{
    ros::init(argc, argv, "PointcloudImage");
    PointCloudImage pci;  // Load class
    ros::spin();
    //std::cout << "Hey";
    //ros::NodeHandle n;
    //ros::Publisher pub = n.advertise<sensor_msgs::Image> ("/yolo/CloudImage",1);
    //ros::Subscriber sub = n.subscribe("/zed2/zed_node/point_cloud/cloud_registered", 1, callback_cloud);
    
    return 0;
}



