#pragma once

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <iostream>

#include <ros/ros.h>

namespace bobnet_config {

using namespace std;

vector<string> split(const string &s, char delim) {
    vector<string> result;
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        result.push_back(item);
    }
    return result;
}

template <typename T>
T fromConfig(const YAML::Node &node, const string &key, const char delim = '/') {
    YAML::Node component(node);
    for (auto &k : split(key, delim)) {
        component = component[k];
    }
    return component.as<T>();
}

template <typename T>
T fromConfigFile(const string &filename, const string &key, const char delim = '/') {
    YAML::Node node = YAML::LoadFile(filename);
    return fromConfig<T>(node, key, delim);
}

YAML::Node loadRosConfigFile() {
    string filename;
    if (!ros::NodeHandle().getParam("/config_file", filename)) {
        ROS_ERROR("Could not get config file path from ROS parameter server!");
        throw runtime_error("Could not get config file path from ROS parameter server!");
    }
    YAML::Node node = YAML::LoadFile(filename);
    return node;
}

template <typename T>
T fromRosConfigFile(const string &key, const char delim = '/') {
    string filename;
    if (!ros::NodeHandle().getParam("/config_file", filename)) {
        ROS_ERROR("Could not get config file path from ROS parameter server!");
        throw runtime_error("Could not get config file path from ROS parameter server!");
    }
    return fromConfigFile<T>(filename, key, delim);
}

}  // namespace bobnet_config