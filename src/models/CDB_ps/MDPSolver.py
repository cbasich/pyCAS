#!/usr/bin/env python
import json
import os
import rospy
import roslib
import pickle
import numpy as np

from message_directory import MDPSolverResponse, MDPSolverRequest

CURRENT_FILE_PATH = os.path.dirname(os.path.realpath(__file__))

MDP_RESPONSE_PUBLISHER = rospy.Publisher("mdp_solver/plan", MDPSolverResponse, queue_size=1)
TASK_STATUS_PUBLISHER = rospy.Publisher("monitor/task_status", TaskStatus, queue_size=1)

current_request_info = None


def update_dataset(edge, p):
    with open(dataset_file_location, mode='rb') as f:
        dataset = pickle.load(f, encoding='bytes')
    dataset.append(edge['s0_id'], edge['s1_id'], p)
    with open(dataset_file_location, mode='wb') as f:
        pickle.dump(dataset, f, protocol = pickle.HIGHEST_PROTOCOL)


# TODO Implement this function
def get_static_map():
    with open('map.json') as world_map_file:
        return json.load(world_map_file)


def ip_info_callback(message):
    new_request = MDPSolverRequest()
    new_request.start = current_request_info.start
    new_request.goal = current_request_info.goal
    
    map_info = json.loads(current_request_info.map_info)
    for edge in map_info['edges']:
        if edge['s0_id'] == message.s0_id and edge['s1_id'] == message.s1_id:
            map_info[edge]['failure_likelihood'] = message.failure_likelihood
            update_dataset(edge, message.failure_likelihood)
            break

    new_request.map_info = map_info
    execute(new_request)


def execute(request):
    rospy.loginfo("Info[mdp_solver_node.execute]: Received a new solver request: %s", request)
    global current_request_info
    current_request_info = request

    map_info = json.loads(request.map)
    init_node_id = request.start
    goal_node_id = request.goal

    base_path_mdp = PathDomainModel(map_info)
    base_path_mdp.set_init(init_node_id)
    base_path_mdp.set_goal(goal_node_id)
    base_path_mdp.reset()

    ps_path_mdp = PerceptionSensitiveMDP(base_path_mdp)
    ps_path_mdp.solve()

    plan = []
    state = ps_path_mdp.init
    while state != ps_path_mdp.goal:
        plan.append(state[0][0])
        state = ps_path_mdp.generate_successor(ps_path_mdp.pi[ps_path_mdp.states.index(states)])
    plan.append(state[0][0])

    response = MDPSolverResponse()
    response.plan = plan
    MDP_RESPONSE_PUBLISHER.publish(response)


def main():
    rospy.loginfo("Info[mdp_solver.main]: Instantiating the mdp_solver node...")
    rospy.init_node("mdp_solver_node", anonymous=True)

    static_map = get_static_map()

    srv = rospy.Service('mdp_solver', MDPSolverRequest, execute)

    # rospy.Subscriber("monitor/introspective_perception", IntrospectivePerceptionInfo, ip_info_callback, queue_size=1)

    rospy.loginfo("Info[mdp_solver_node.main]: Spinning...")
    rospy.spin()


if __name__ == '__main__':
    main()