#pragma once

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include <iostream>

#include <ros/ros.h>

#include <bobnet_core/Types.h>

namespace bobnet_config {

using namespace std;
using namespace bobnet_core;

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
T parseNode(const YAML::Node &node) {
    return node.as<T>();
}

template <>
matrix_t parseNode(const YAML::Node &node) {
    if (node.size() == 0 || node[0].size() == 0) {
        std::cerr << "Cannot parse empty matrix" << std::endl;
        throw runtime_error("Empty matrix!");
    }

    matrix_t output(node.size(), node[0].size());
    for (int i = 0; i < node.size(); i++) {
        for (int j = 0; j < node[i].size(); j++) {
            output(i, j) = node[i][j].as<scalar_t>();
        }
    }
    return output;
}

template <>
vector_t parseNode(const YAML::Node &node) {
    if (node.size() == 0) {
        std::cerr << "Cannot parse empty vector" << std::endl;
        throw runtime_error("Empty vector!");
    }

    vector_t output(node.size());
    for (int i = 0; i < node.size(); i++) {
        output(i) = node[i].as<scalar_t>();
    }
    return output;
}

template <typename T>
T fromConfig(const YAML::Node &node, const string &key, const char delim = '/') {
    if (node.size() == 0) {
        std::cerr << "Empty config file! Make sure not to use same node twice!" << std::endl;
        throw runtime_error("Empty config file!");
    }

    YAML::Node component(node);
    for (auto &k : split(key, delim)) {
        component = component[k];
    }

    return parseNode<T>(component);
}

template <typename T>
T fromConfigFile(const string &filename, const string &key, const char delim = '/') {
    YAML::Node node = YAML::LoadFile(filename);
    return fromConfig<T>(node, key, delim);
}

template <typename T>
T fromRosConfig(const string &key, const char delim = '/') {
    string filename;
    if (!ros::NodeHandle().getParam("/config_file", filename)) {
        ROS_ERROR("Could not get config file path from ROS parameter server!");
        throw runtime_error("Could not get config file path from ROS parameter server!");
    }
    return fromConfigFile<T>(filename, key, delim);
}

}  // namespace bobnet_config