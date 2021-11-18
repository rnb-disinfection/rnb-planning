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
            auto svc = static_cast<const ompl::base::StateValidityChecker*>(si_->getStateValidityChecker().get()); // si_->getStateValidityChecker();
            auto cl = svc->clearance(s);
//            std::cout <<"clearance: " << cl  << std::endl;
            return ompl::base::Cost(1 / (cl+1e-3));
        }
    };

    /*! @brief get combined objective of clearance & path length
     */
    ompl::base::OptimizationObjectivePtr getBalancedObjective(const ompl::base::SpaceInformationPtr& si,
                                                              double max_cost = 1e5)
    {
        ompl::base::OptimizationObjectivePtr lengthObj = std::make_shared<ompl::base::PathLengthOptimizationObjective>(si);
        ompl::base::OptimizationObjectivePtr clearObj = std::make_shared<ClearanceObjective>(si);

        std::shared_ptr<ompl::base::MultiOptimizationObjective> opt = std::make_shared<ompl::base::MultiOptimizationObjective>(si);
        opt->addObjective(lengthObj, 0.1);
        opt->addObjective(clearObj, 1.0);
        opt->setCostThreshold(ompl::base::Cost(max_cost));
        return ompl::base::OptimizationObjectivePtr(opt);
    }

    /*! @brief get length objective of clearance & path length
     */
    ompl::base::OptimizationObjectivePtr getLengthObjective(const ompl::base::SpaceInformationPtr& si,
                                                            double max_cost = 1e5)
    {
        ompl::base::OptimizationObjectivePtr lengthObj = std::make_shared<ompl::base::PathLengthOptimizationObjective>(si);
        lengthObj->setCostThreshold(ompl::base::Cost(max_cost));
        return lengthObj;
    }

}

#endif //MOVEIT_INTERFACE_PY_PLANNING_OBJECTIVE_H
