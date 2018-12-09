import Queue
import numpy as np
import Utils

def generate_plan (prm, source, target):
    if not prm.add_vertex_to_graph (source):
        return None

    if not prm.add_vertex_to_graph (target):
        return None

    source = tuple (source)
    target = tuple (target)

    queue = Queue.PriorityQueue ()

    queue.put (source, 0)

    past_nodes = {}
    curr_cost = {}

    past_nodes[source] = None
    curr_cost[source] = 0

    while not queue.empty ():
        curr_node = queue.get ()

        if curr_node == target:
            break

        for next_node in [n for n in prm.graph.neighbors (curr_node) if n != curr_node]:
            new_cost = curr_cost[curr_node] + prm.graph.get_edge_data (curr_node, next_node)['weight']

            if next_node not in curr_cost or new_cost < curr_cost[next_node]:
                curr_cost[next_node] = new_cost
                priority = new_cost + Utils.euclidean_cost (curr_node, next_node)
                queue.put (next_node, priority)
                past_nodes[next_node] = curr_node

    curr_node = target
    plan = [target]

    while past_nodes and curr_node != source:
        curr_node = past_nodes.pop (curr_node)
        plan.append (curr_node)

    plan.reverse()
    return plan