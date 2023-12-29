#include <bobnet_gridmap/StaticLoader.h>

#include <string>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "bobnet_gridmap");
    ros::NodeHandle nh;

    std::string topic;
    if (!nh.getParam("/bobnet/gridmap_topic", topic)) {
        ROS_WARN_STREAM("Could not get gridmap_topic parameter, using default value: " << topic);
        throw std::runtime_error("Could not get gridmap_topic parameter");
    }

    std::string mapPath;
    if (!nh.getParam("/bobnet/gridmap_path", mapPath)) {
        ROS_WARN_STREAM("Could not get gridmap_path parameter, using default value: " << mapPath);
        throw std::runtime_error("Could not get gridmap_path parameter");
    }

    bobnet_gridmap::scalar_t resolution = 0.04;
    const std::string mapFrame = "odom";

    bobnet_gridmap::StaticLoader loader(mapPath, resolution, mapFrame, topic);
    loader.publish();
    ros::spin();

    return EXIT_SUCCESS;
}