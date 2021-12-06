
##
# @class CustomRuleInterface
# @brief interface class for custom rules in planning
class CustomRuleInterface:
    ##
    # @param pscene rnb-planning.src.pkg.planning.scene.PlanningScene
    def __init__(self, pscene):
        pass

    ##
    # @param tplan sub-class of rnb-planning.src.pkg.planning.task.interface.TaskInterface
    def init(self, tplan, multiprocess_manager):
        pass
