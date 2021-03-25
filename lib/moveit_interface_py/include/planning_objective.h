//
// Created by rnb on 21. 3. 25..
//

#ifndef MOVEIT_INTERFACE_PY_PLANNING_OBJECTIVE_H
#define MOVEIT_INTERFACE_PY_PLANNING_OBJECTIVE_H

#include <ompl/base/Constraint.h>
#include "ompl/base/objectives/StateCostIntegralObjective.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"

namespace RNB {

    /*! \class ClearanceObjective
     * @brief clearance objective, copied from OMPL tutorial example
     */
    class ClearanceObjective : public ompl::base::StateCostIntegralObjective {
    public:
        ClearanceObjective(const ompl::base::SpaceInformationPtr &si) :
                ompl::base::StateCostIntegralObjective(si, true) {
        }

        ompl::base::Cost stateCost(const ompl::base::State *s) const {
            return ompl::base::Cost(1 / si_->getStateValidityChecker()->clearance(s));
        }
    };

    /*! @brief get combined objective of clearance & path length
     */
    ompl::base::OptimizationObjectivePtr getBalancedObjective(const ompl::base::SpaceInformationPtr& si)
    {
        ompl::base::OptimizationObjectivePtr lengthObj = std::make_shared<ompl::base::PathLengthOptimizationObjective>(si);
        ompl::base::OptimizationObjectivePtr clearObj = std::make_shared<ClearanceObjective>(si);

        std::shared_ptr<ompl::base::MultiOptimizationObjective> opt = std::make_shared<ompl::base::MultiOptimizationObjective>(si);
        opt->addObjective(lengthObj, 0.001);
        opt->addObjective(clearObj, 1.0);
        opt->setCostThreshold(ompl::base::Cost(5));
        return ompl::base::OptimizationObjectivePtr(opt);
    }

    /*! @brief get length objective of clearance & path length
     */
    ompl::base::OptimizationObjectivePtr getLengthObjective(const ompl::base::SpaceInformationPtr& si,
                                                            double max_cost = 3.14*6)
    {
        ompl::base::OptimizationObjectivePtr lengthObj = std::make_shared<ompl::base::PathLengthOptimizationObjective>(si);
        lengthObj->setCostThreshold(ompl::base::Cost(max_cost));
        return lengthObj;
    }

}

#endif //MOVEIT_INTERFACE_PY_PLANNING_OBJECTIVE_H
