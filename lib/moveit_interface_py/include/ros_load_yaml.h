//
// Created by tamp on 20. 11. 27..
//

#ifndef MOVEIT_PLAN_COMPACT_ROS_LOAD_YAML_H
#define MOVEIT_PLAN_COMPACT_ROS_LOAD_YAML_H
#include <string>
#include <logger.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include "constants.h"

namespace RNB{
    using namespace std;
    XmlRpc::XmlRpcValue get_rosparam_from_yaml_node(YAML::Node node){
        if (node.IsMap()){
            XmlRpc::XmlRpcValue map;
            for(auto it=node.begin(); it!=node.end(); it++){
                map[it->first.as<string>()] = get_rosparam_from_yaml_node(it->second);
            }
            return map;
        }
        else if (node.IsSequence()) {
            XmlRpc::XmlRpcValue seq;
            seq.setSize(node.size());
            int idx=0;
            for(auto it=node.begin(); it!=node.end(); it++){
                seq[idx] = get_rosparam_from_yaml_node(*it);
                idx++;
            }
            return seq;
        }
        else{
            try {
                return XmlRpc::XmlRpcValue(node.as<int>());
            } catch (...) {
                try {
                    return XmlRpc::XmlRpcValue(node.as<double>());
                } catch (...) {
                    try {
                        return XmlRpc::XmlRpcValue(node.as<bool>());
                    } catch (...) {
                        try {
                            return XmlRpc::XmlRpcValue(node.as<string>());
                        } catch (...) {
                            PRINT_ERROR("CANNOT CONVERT YAML NODE TO ROSPARAM");
                            exit(0);
                        }
                    }
                }
            }
        }
    }

    void set_ros_pram_recurse(ros::NodeHandlePtr _node_handle, string _key, XmlRpc::XmlRpcValue _node){
        if(_node.getType()==XmlRpc::XmlRpcValue::TypeStruct){
            for(auto it=_node.begin(); it!=_node.end(); it++){
                auto sub_key = it->first;
                auto sub_node = it->second;
                set_ros_pram_recurse(_node_handle, _key+"/"+sub_key, sub_node);
            }
        }
        else{
            _node_handle->setParam(_key, _node);
        }
    }

    void rosparam_load_yaml(ros::NodeHandlePtr _node_handle, string _node_name, string filname){
        std::ifstream file_in(filname);
        if(!file_in){
            PRINT_ERROR(("Put config file - " + filname).c_str());
        }
        YAML::Node doc = YAML::Load(file_in);

        XmlRpc::XmlRpcValue nodeval = get_rosparam_from_yaml_node(doc);
        set_ros_pram_recurse(_node_handle, _node_name, nodeval);
    }
}

#endif //MOVEIT_PLAN_COMPACT_ROS_LOAD_YAML_H
