#include "MyHW6.h"
#include "AMPCore.h"

MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem &problem, const amp::SearchHeuristic &heuristic)
{
    GraphSearchResult path;

    MyNode starting_node;
    starting_node.nodeID = problem.init_node;
    starting_node.pathCost = 0;
    starting_node.priority = starting_node.pathCost + heuristic(starting_node.nodeID);
    open_list.push_back(starting_node);
    MyNode ending_node;
    ending_node.nodeID = problem.goal_node;
    LOG("START NODE: ID=" << starting_node.nodeID);
    vector<amp::Node> test_child = problem.graph->children(starting_node.nodeID);

    MyNode current_node;
    int cnt = 0;
    int break_cnt = 15000;
    while (open_list.size() > 0)
    {
        current_node = open_list.front(); // pick n_best
        LOG("Popped Node: " << current_node.nodeID);
        closed_list.push_back(current_node); // add n_best to C
        open_list.erase(open_list.begin());  // remove n_bast from O

        if (current_node.nodeID == ending_node.nodeID)
        {
            LOG("GOAL REACHED! Total iterations: " << cnt);
            path.success = true;
            break;
        }

        MyNode i_node;
        bool in_closed_list;
        bool in_open_list;
        vector<amp::Node> childrenID = problem.graph->children(current_node.nodeID);
        vector<double> childrenPathCost = problem.graph->outgoingEdges(current_node.nodeID);
        for (int i_child = 0; i_child < childrenID.size(); i_child++)
        {
            i_node.nodeID = childrenID[i_child];
            i_node.pathCost = current_node.pathCost + childrenPathCost[i_child];
            i_node.priority = i_node.pathCost + heuristic(i_node.nodeID);
            i_node.parent = current_node.nodeID;
            LOG("ITERATION: " << i_child);
            LOG("NodeID = " << i_node.nodeID << " | Path Cost = " << i_node.pathCost << " | Priority = " << i_node.priority);

            in_closed_list = isNodeInList(i_node, closed_list);
            in_open_list = isNodeInList(i_node, open_list);
            if (!in_closed_list)
            {
                if (!in_open_list)
                {
                    open_list.push_back(i_node);
                }
                else
                {
                    LOG("Caveat triggered...");
                    MyNode previous_node = findNodeByID(open_list, i_node.nodeID);
                    LOG("PREVIOUS: NodeID = " << previous_node.nodeID << " | Path Cost = " << previous_node.pathCost << " | Priority = " << previous_node.priority);
                    // double best_iNode_priority = open_list[i_node.nodeID].priority;
                    if (i_node.pathCost < (previous_node.pathCost))
                    {
                        removeNodeByID(open_list, previous_node.nodeID);
                        open_list.push_back(i_node);
                        LOG("NEW: NodeID = " << i_node.nodeID << " | Path Cost = " << i_node.pathCost << " | Priority = " << i_node.priority);
                        LOG("Caveat completed.");
                    }
                    // LOG("Caveat triggered...");
                    // MyNode previous_node = findNodeByID(open_list, i_node.nodeID);
                    // LOG("PREVIOUS: NodeID = " << previous_node.nodeID << " | Path Cost = " << previous_node.pathCost << " | Priority = " << previous_node.priority);
                    // // double best_iNode_priority = open_list[i_node.nodeID].priority;
                    // if (i_node.pathCost < (previous_node.pathCost))
                    // {
                    //     // MyNode best_node = findNodeByID(open_list, i_node.nodeID);
                    //     // i_node.pathCost = previous_node.pathCost;
                    //     // i_node.priority = previous_node.priority;
                    //     // i_node.parent = previous_node.parent;
                    //     removeNodeByID(open_list, previous_node.nodeID);
                    //     previous_node.pathCost = i_node.pathCost;
                    //     previous_node.priority = i_node.priority;
                    //     previous_node.parent = i_node.parent;
                    //     // open_list.erase(previous_node.nodeID);
                    //     open_list.push_back(previous_node);
                    //     LOG("PREVIOUS: NodeID = " << previous_node.nodeID << " | Path Cost = " << previous_node.pathCost << " | Priority = " << previous_node.priority);

                    //     LOG("Caveat completed.");
                    // }
                }
            }
        }
        // LOG("Pre-sorted open list:");
        // for (int ii = 0; ii < open_list.size(); ii++)
        // {
        //     LOG(open_list[ii].nodeID);
        // }

        sortOpenList(open_list);

        // LOG("Post-sorted open list:");
        // for (int ii = 0; ii < open_list.size(); ii++)
        // {
        //     LOG(open_list[ii].nodeID);
        // }

        // if (cnt > break_cnt)
        // {
        //     LOG("Main while Loop break...");
        //     break;
        // }
        // cnt++;
    }

    LOG("FINAL CLOSED LIST:");

    MyNode copy_node;
    amp::Node copy_node_id = ending_node.nodeID;
    cnt = 0;
    while (!(copy_node_id == starting_node.nodeID))
    {
        path.node_path.push_front(copy_node_id);
        LOG(copy_node_id);
        copy_node = findNodeByID(closed_list, copy_node_id);
        copy_node_id = copy_node.parent;
        // LOG("parent->" << copy_node_id);
        if (cnt > break_cnt)
        {
            LOG("Copy while Loop break...");
            break;
        }
        cnt++;
    }
    path.node_path.push_front(starting_node.nodeID);

    MyNode the_last_node = findNodeByID(closed_list, ending_node.nodeID);
    path.path_cost = the_last_node.pathCost;
    LOG("TOTAL PATH LENGTH: " << path.path_cost);
    LOG("START NODE: ID=" << starting_node.nodeID);
    LOG("END NODE: ID=" << ending_node.nodeID);

    return path;
}

void MyAStarAlgo::removeNodeByID(std::vector<MyNode> &alist, const amp::Node &targetNodeID)
{
    std::remove_if(alist.begin(), alist.end(), [targetNodeID](const MyNode &node)
                   { return node.nodeID == targetNodeID; });
    // alist.erase(
    //     ),
    //     alist.end());
}

// Function to check if new_child.nodeID is in open_list
bool MyAStarAlgo::isNodeInList(const MyNode &achild, const std::vector<MyNode> &alist)
{
    for (const MyNode &i_node : alist)
    {
        if (i_node.nodeID == achild.nodeID)
        {
            return true; // Found a match
        }
    }
    return false; // No match found
}

MyNode MyAStarAlgo::findNodeByID(const std::vector<MyNode> &a_list, const amp::Node &targetNodeID)
{
    for (const MyNode &i_node : a_list)
    {
        if (i_node.nodeID == targetNodeID)
        {
            return i_node; // Found the matching node
        }
        // else{
        //     LOG("findNodeByID Error...this should not be happening");
        // }
    }
}

void MyAStarAlgo::sortOpenList(std::vector<MyNode> &nodes)
{
    std::sort(nodes.begin(), nodes.end(), [](const MyNode &a, const MyNode &b)
              { return a.priority < b.priority; });
}

// bool MyAStarAlgo::compareByPriority(const MyNode &a, const MyNode &b)
// {
//     return a.priority < b.priority;
// }