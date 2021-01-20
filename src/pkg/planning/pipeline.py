##
# @class    PlanningPipeline
# @brief    planning pipeline
class PlanningPipeline:
    ##
    # @param mplan subclass instance of rnb-planning.src.pkg.planning.motion.interface.MotionInterface
    def set_motion(self, mplan):
        self.mplan = mplan
        mplan.update(self)

    ##
    # @param tplan subclass instance of rnb-planning.src.pkg.planning.task.interface.TaskInterface
    def set_sampler(self, tplan):
        self.tplan = tplan