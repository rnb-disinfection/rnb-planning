/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Dave Coleman */

#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/profiler/profiler.h>
#include <class_loader/class_loader.hpp>

#include <dynamic_reconfigure/server.h>
#include "moveit_planners_ompl/OMPLDynamicReconfigureConfig.h"

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <ompl/util/Console.h>

#include <thread>
#include <memory>

namespace ompl_interface
{
    using namespace moveit_planners_ompl;

    constexpr char LOGNAME[] = "ompl_planner_manager";

#define OMPL_ROS_LOG(ros_log_level)                                                                                    \
  {                                                                                                                    \
    ROSCONSOLE_DEFINE_LOCATION(true, ros_log_level, ROSCONSOLE_NAME_PREFIX ".ompl");                                   \
    if (ROS_UNLIKELY(__rosconsole_define_location__enabled))                                                           \
      ::ros::console::print(0, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_,    \
                            filename, line, __ROSCONSOLE_FUNCTION__, "%s", text.c_str());                              \
  }

    class OMPLPlannerManagerCustom : public planning_interface::PlannerManager
    {
    public:
        OMPLPlannerManagerCustom() : planning_interface::PlannerManager(), nh_("~")
        {
            class OutputHandler : public ompl::msg::OutputHandler
            {
            public:
                void log(const std::string& text, ompl::msg::LogLevel level, const char* filename, int line) override
                {
                    switch (level)
                    {
                        case ompl::msg::LOG_DEV2:
                        case ompl::msg::LOG_DEV1:
                        case ompl::msg::LOG_DEBUG:
                        OMPL_ROS_LOG(::ros::console::levels::Debug);
                            break;
                        case ompl::msg::LOG_INFO:
                        OMPL_ROS_LOG(::ros::console::levels::Info);
                            break;
                        case ompl::msg::LOG_WARN:
                        OMPL_ROS_LOG(::ros::console::levels::Warn);
                            break;
                        case ompl::msg::LOG_ERROR:
                        OMPL_ROS_LOG(::ros::console::levels::Error);
                            break;
                        case ompl::msg::LOG_NONE:
                        default:
                            /* ignore */
                            break;
                    }
                }
            };

            output_handler_.reset(new OutputHandler());
            ompl::msg::useOutputHandler(output_handler_.get());
        }

        bool initialize(const moveit::core::RobotModelConstPtr& model, const std::string& ns) override
        {
            if (!ns.empty())
                nh_ = ros::NodeHandle(ns);
            ompl_interface_.reset(new OMPLInterface(model, nh_));
            std::string ompl_ns = ns.empty() ? "ompl" : ns + "/ompl";
            dynamic_reconfigure_server_.reset(
                    new dynamic_reconfigure::Server<OMPLDynamicReconfigureConfig>(ros::NodeHandle(nh_, ompl_ns)));
            dynamic_reconfigure_server_->setCallback(
                    std::bind(&OMPLPlannerManagerCustom::dynamicReconfigureCallback, this, std::placeholders::_1, std::placeholders::_2));
            config_settings_ = ompl_interface_->getPlannerConfigurations();
            return true;
        }

        bool canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const override
        {
            return req.trajectory_constraints.constraints.empty();
        }

        std::string getDescription() const override
        {
            return "OMPL";
        }

        void getPlanningAlgorithms(std::vector<std::string>& algs) const override
        {
            const planning_interface::PlannerConfigurationMap& pconfig = ompl_interface_->getPlannerConfigurations();
            algs.clear();
            algs.reserve(pconfig.size());
            for (const std::pair<const std::string, planning_interface::PlannerConfigurationSettings>& config : pconfig)
                algs.push_back(config.first);
        }

        void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pconfig) override
        {
            // this call can add a few more configs than we pass in (adds defaults)
            ompl_interface_->setPlannerConfigurations(pconfig);
            // so we read the configs instead of just setting pconfig
            PlannerManager::setPlannerConfigurations(ompl_interface_->getPlannerConfigurations());
        }

        planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                                  const planning_interface::MotionPlanRequest& req,
                                                                  moveit_msgs::MoveItErrorCodes& error_code) const override
        {
            return ompl_interface_->getPlanningContext(planning_scene, req, error_code);
        }

    private:
        void dynamicReconfigureCallback(OMPLDynamicReconfigureConfig& config, uint32_t /*level*/)
        {
            if (config.link_for_exploration_tree.empty() && !planner_data_link_name_.empty())
            {
                pub_markers_.shutdown();
                planner_data_link_name_.clear();
                ROS_INFO_NAMED(LOGNAME, "Not displaying OMPL exploration data structures.");
            }
            else if (!config.link_for_exploration_tree.empty() && planner_data_link_name_.empty())
            {
                pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("ompl_planner_data_marker_array", 5);
                planner_data_link_name_ = config.link_for_exploration_tree;
                ROS_INFO_NAMED(LOGNAME, "Displaying OMPL exploration data structures for %s", planner_data_link_name_.c_str());
            }

            ompl_interface_->simplifySolutions(config.simplify_solutions);
            ompl_interface_->getPlanningContextManager().setMaximumSolutionSegmentLength(config.maximum_waypoint_distance);
            ompl_interface_->getPlanningContextManager().setMinimumWaypointCount(config.minimum_waypoint_count);
            if (display_random_valid_states_ && !config.display_random_valid_states)
            {
                display_random_valid_states_ = false;
                if (pub_valid_states_thread_)
                {
                    pub_valid_states_thread_->join();
                    pub_valid_states_thread_.reset();
                }
                pub_valid_states_.shutdown();
                pub_valid_traj_.shutdown();
            }
            else if (!display_random_valid_states_ && config.display_random_valid_states)
            {
                pub_valid_states_ = nh_.advertise<moveit_msgs::DisplayRobotState>("ompl_planner_valid_states", 5);
                pub_valid_traj_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("ompl_planner_valid_trajectories", 5);
                display_random_valid_states_ = true;
                //    pub_valid_states_thread_.reset(new boost::thread(boost::bind(&OMPLPlannerManager::displayRandomValidStates,
                //    this)));
            }
        }

        ros::NodeHandle nh_;
        std::unique_ptr<dynamic_reconfigure::Server<OMPLDynamicReconfigureConfig>> dynamic_reconfigure_server_;
        std::unique_ptr<OMPLInterface> ompl_interface_;
        std::unique_ptr<std::thread> pub_valid_states_thread_;
        bool display_random_valid_states_{ false };
        ros::Publisher pub_markers_;
        ros::Publisher pub_valid_states_;
        ros::Publisher pub_valid_traj_;
        std::string planner_data_link_name_;
        std::shared_ptr<ompl::msg::OutputHandler> output_handler_;
    };

}  // namespace ompl_interface

CLASS_LOADER_REGISTER_CLASS(ompl_interface::OMPLPlannerManagerCustom, planning_interface::PlannerManager);
