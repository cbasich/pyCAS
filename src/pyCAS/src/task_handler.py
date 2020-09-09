from models.CDB.domain_model import DeliveryBotDomain
from models.CDB.autonomy_model import AutonomyModel
from models.CDB.feedback_model import FeedbackModel
from models.CDB.competence_aware_system import CAS

class CASTaskHandler(object):
    def get_state(self, message):
        return (message.robot_status.x_coord, message.robot_status.y_coord)

    def is_goal(self, state, goal):
        return state == goal

    def get_solution(self, world_map, start, goal):
    # world_map = json.load()
    # end = message.goal
    # start = task_handler.get_state_from_message(ssp_state_message)

        rospy.loginfo("Info[CDB_execution_node.instantiate]: Instantiating the domain model...")
        DM = DeliveryBotDomain(world_map, start, goal)

        rospy.loginfo("Info[CDB_execution_node.instantiate]: Instantiating the autonomy model...")
        AM = AutonomyModel(DM, [0,1,2,3,])

        rospy.loginfo("Info[CDB_execution_node.instantiate]: Instantiating the feedback model...")
        HM = FeedbackModel(DM, AM, ['+', '-', '/', None], ['open'])

        rospy.loginfo("Info[CDB_execution_node.instantiate]: Instanaitating the CAS model...")
        solution = CAS(DM, AM, HM, persistence = 0.75)

        solution.solve()
        policy = solution.pi 
        return policy