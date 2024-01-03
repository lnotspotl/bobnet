#include <bobnet_gridmap/StaticLoader.h>
#include <bobnet_config/utils.h>

#include <string>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "bobnet_gridmap");
    ros::NodeHandle nh;


    std::string mapPath;
    if (!nh.getParam("/bobnet/gridmap_path", mapPath)) {
        throw std::runtime_error("Could not get gridmap_path parameter");
    }

    auto resolution = bobnet_config::fromRosConfigFile<double>("gridmap_resolution");
    auto mapFrame = bobnet_config::fromRosConfigFile<std::string>("odom_frame");

    auto gridmapTopic = bobnet_config::fromRosConfigFile<std::string>("gridmap_topic");
    bobnet_gridmap::StaticLoader loader(mapPath, resolution, mapFrame, gridmapTopic);
    loader.publish();
    ros::spin();

    return EXIT_SUCCESS;
}