#include <bobnet_gridmap/StaticLoader.h>

#include <pcl/io/vtk_lib_io.h>
#include <grid_map_pcl/GridMapPclConverter.hpp>


namespace bobnet_gridmap {

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
StaticLoader::StaticLoader(scalar_t resolution, const std::string &map_frame, const std::string &topic)
    : resolution_(resolution), map_frame_(map_frame) {
    publisher_ = ros::NodeHandle().advertise<grid_map_msgs::GridMap>(topic, 1, true);
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
StaticLoader::StaticLoader(const std::string &path, scalar_t resolution,
                           const std::string &map_frame, const std::string &topic)
    : StaticLoader(resolution, map_frame, topic) {
    load(path);
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void StaticLoader::load(const std::string &path) {
    pcl::PolygonMesh mapMesh;
    pcl::io::loadPolygonFilePLY(path, mapMesh);

    // Set map tf_frame
    map_.setFrameId(map_frame_);

    // Set map resolution
    grid_map::GridMapPclConverter::initializeFromPolygonMesh(mapMesh, resolution_, map_);

    // Load elevation layer
    const std::string elevationLayer = "elevation";
    grid_map::GridMapPclConverter::addLayerFromPolygonMesh(mapMesh, elevationLayer, map_);

    initialized_ = true;
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void StaticLoader::publish() {
    if (!isInitialized()) {
        ROS_WARN("[Bobnet gridmap] Cannot publish uninitialized map.");
        return;
    }

    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(map_, message);
    publisher_.publish(message);
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void StaticLoader::publishPeriodically(scalar_t rate) {
    ros::Rate r(rate);
    while (ros::ok()) {
        publish();
        r.sleep();
    }
}

}  // namespace bobnet_gridmap