#include "bobnet_config/utils.h"

vector<string> split(const string &s, char delim) {
    vector<string> result;
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        result.push_back(item);
    }
    return result;
}

#define PARSE_NODE(TYPE) \
TYPE parseNode(const YAML::Node &node) { \
    return node.as<TYPE>(); \
} \

PARSE_NODE(scalar_t)
PARSE_NODE(vector_t)
PARSE_NODE(matrix_t)
PARSE_NODE(vector<string>)

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

#define FROM_CONFIG(TYPE) \
TYPE fromConfig(const YAML::Node &node, const string &key, const char delim) { \
    if (node.size() == 0) { \
        std::cerr << "Empty config file! Make sure not to use same node twice!" << std::endl; \
        throw runtime_error("Empty config file!"); \
    } \
    \
    YAML::Node component(node); \
    for (auto &k : split(key, delim)) { \
        component = component[k]; \
    } \
    \
    return parseNode<TYPE>(component); \
} \

FROM_CONFIG(scalar_t)
FROM_CONFIG(vector_t)
FROM_CONFIG(matrix_t)
FROM_CONFIG(vector<string>)

#define FROM_CONFIG_FILE(TYPE) \
TYPE fromRosConfigFile(const string &filename, const string &key, const char delim) { \
    YAML::Node node = YAML::LoadFile(filename); \
    return fromConfig<TYPE>(node, key, delim); \
} \

FROM_CONFIG_FILE(scalar_t)
FROM_CONFIG_FILE(vector_t)
FROM_CONFIG_FILE(matrix_t)
FROM_CONFIG_FILE(vector<string>)