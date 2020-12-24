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

namespace RNB{
    using namespace std;

    template<typename T>
    void set_rosparam_array_as(ros::NodeHandlePtr _node_handle, string key, YAML::Node node){
        vector<T> seq;
        for(auto it=node.begin(); it!=node.end(); it++){
            auto node_T = it->as<T>();
            seq.push_back(node_T);
        }
        _node_handle->setParam(key, seq);
        cout<<"rosparam set: "<< key << ": " << endl;
        for(auto it=seq.begin(); it!=seq.end(); it++){
            cout<< *it << ", ";
        }
        cout<< endl;
    }

    template<typename T>
    void set_rosparam_as(ros::NodeHandlePtr _node_handle, string key, YAML::Node node){
        auto node_T = node.as<T>();
        _node_handle->setParam(key, node_T);
        cout<<"rosparam set: "<< key << ": " << node_T << endl;
    }

    void set_rosparam_from_yaml_node(ros::NodeHandlePtr _node_handle, string key, YAML::Node node){
        if (node.IsSequence()){
            try{
                set_rosparam_array_as<int>(_node_handle, key, node);
            } catch (...) {
                try{
                    set_rosparam_array_as<double>(_node_handle, key, node);
                } catch (...) {
                    try{
                        set_rosparam_array_as<string>(_node_handle, key, node);
                    } catch (...) {
                        PRINT_ERROR("CANNOT CONVERT YAML ARRAY NODE TO ROSPARAM ("+key+")");
                        exit(0);
                    }

                }

            }
        }
        else{
            try{
                set_rosparam_as<int>(_node_handle, key, node);
            } catch (...) {
                try{
                    set_rosparam_as<double>(_node_handle, key, node);
                } catch (...) {
                    try{
                        set_rosparam_as<string>(_node_handle, key, node);
                    } catch (...) {
                        PRINT_ERROR("CANNOT CONVERT YAML NODE TO ROSPARAM ("+key+")");
                        exit(0);
                    }

                }

            }
        }
    }

    void recurse_set_param(ros::NodeHandlePtr _node_handle, string key_prev, YAML::Node::iterator it_doc, YAML::Node::iterator it_end){
        if (it_doc==it_end){
            return;
        }
        stringstream stream1;
        if (key_prev.size()==0) {
            stream1<< it_doc->first;
        }
        else{
            stream1<<key_prev<<"/"<< it_doc->first;
        }
        string current_key = stream1.str();
        if(it_doc->second.IsMap()) {
            recurse_set_param(_node_handle, current_key, it_doc->second.begin(), it_doc->second.end());
        }
        else if(it_doc->second.IsDefined()){
            set_rosparam_from_yaml_node(_node_handle, current_key, it_doc->second);
        }
        else{
            PRINT_ERROR("ERROR::NON_DEFINED NODE" + current_key);
            exit(0);
        }
        recurse_set_param(_node_handle, key_prev, ++it_doc, it_end);
    }

    void rosparam_load_yaml(ros::NodeHandlePtr _node_handle, string _node_name, string filname){
        std::ifstream file_in(filname);
        if(!file_in){
            PRINT_ERROR(("Put config file - " + filname).c_str());
        }
        YAML::Node doc = YAML::Load(file_in);
        recurse_set_param(_node_handle, _node_name, doc.begin(), doc.end());
    }
}

#endif //MOVEIT_PLAN_COMPACT_ROS_LOAD_YAML_H
