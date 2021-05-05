
#include "ompl_interface/planner/KPIECE_CUSTOM.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <cassert>

ompl::geometric::KPIECE_CUSTOM::KPIECE_CUSTOM(const base::SpaceInformationPtr &si)
        : base::Planner(si, "KPIECE_CUSTOM"), disc_([this](Motion *m) { freeMotion(m); })
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &KPIECE_CUSTOM::setRange, &KPIECE_CUSTOM::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &KPIECE_CUSTOM::setGoalBias, &KPIECE_CUSTOM::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("border_fraction", this, &KPIECE_CUSTOM::setBorderFraction, &KPIECE_CUSTOM::getBorderFraction,
                                  "0.:0.05:1.");
    Planner::declareParam<double>("failed_expansion_score_factor", this, &KPIECE_CUSTOM::setFailedExpansionCellScoreFactor,
                                  &KPIECE_CUSTOM::getFailedExpansionCellScoreFactor);
    Planner::declareParam<double>("min_valid_path_fraction", this, &KPIECE_CUSTOM::setMinValidPathFraction,
                                  &KPIECE_CUSTOM::getMinValidPathFraction);
}

ompl::geometric::KPIECE_CUSTOM::~KPIECE_CUSTOM() = default;

void ompl::geometric::KPIECE_CUSTOM::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configureProjectionEvaluator(projectionEvaluator_);
    sc.configurePlannerRange(maxDistance_);

    if (failedExpansionScoreFactor_ < std::numeric_limits<double>::epsilon() || failedExpansionScoreFactor_ > 1.0)
        throw Exception("Failed expansion cell score factor must be in the range (0,1]");
    if (minValidPathFraction_ < std::numeric_limits<double>::epsilon() || minValidPathFraction_ > 1.0)
        throw Exception("The minimum valid path fraction must be in the range (0,1]");

    disc_.setDimension(projectionEvaluator_->getDimension());
}

void ompl::geometric::KPIECE_CUSTOM::clear()
{
    Planner::clear();
    sampler_.reset();
    disc_.clear();
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::KPIECE_CUSTOM::freeMotion(Motion *motion)
{
    if (motion->state != nullptr)
        si_->freeState(motion->state);
    delete motion;
}

ompl::base::PlannerStatus ompl::geometric::KPIECE_CUSTOM::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    Discretization<Motion>::Coord xcoord(projectionEvaluator_->getDimension());

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        projectionEvaluator_->computeCoordinates(motion->state, xcoord);
        disc_.addMotion(motion, xcoord, 1.0);
    }

    if (disc_.getMotionCount() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(),
                disc_.getMotionCount());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    base::State *xstate = si_->allocState();

    while (!ptc)
    {
        disc_.countIteration();

        /* Decide on a state to expand from */
        Motion *existing = nullptr;
        Discretization<Motion>::Cell *ecell = nullptr;
        disc_.selectMotion(existing, ecell);
        assert(existing);

        /* sample random state (with goal biasing) */
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(xstate);
        else
            sampler_->sampleUniformNear(xstate, existing->state, maxDistance_);

        std::pair<base::State *, double> fail(xstate, 0.0);
        bool keep = si_->checkMotion(existing->state, xstate, fail);
        if (!keep && fail.second > minValidPathFraction_)
            keep = true;

        if (keep)
        {
            /* create a motion */
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, xstate);
            motion->parent = existing;

            double dist = 0.0;
            bool solv = goal->isSatisfied(motion->state, &dist);
            projectionEvaluator_->computeCoordinates(motion->state, xcoord);
            disc_.addMotion(motion, xcoord, dist);  // this will also update the discretization heaps as needed, so no
            // call to updateCell() is needed

            if (solv)
            {
                approxdif = dist;
                solution = motion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = motion;
            }
        }
        else
            ecell->data->score *= failedExpansionScoreFactor_;
        disc_.updateCell(ecell);
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    si_->freeState(xstate);

    OMPL_INFORM("%s: Created %u states in %u cells (%u internal + %u external)", getName().c_str(),
                disc_.getMotionCount(), disc_.getCellCount(), disc_.getGrid().countInternal(),
                disc_.getGrid().countExternal());

    return {solved, approximate};
}

void ompl::geometric::KPIECE_CUSTOM::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);
    disc_.getPlannerData(data, 0, true, lastGoalMotion_);
}