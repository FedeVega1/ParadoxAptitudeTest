#include "PathFinder.h"
#include <queue>
#include <cmath>

// Start: Starting Point - >= 0 && < MapDimensions && IsWalkeable
// Target: Target Point - >= 0 && < MapDimensions && IsWalkeable
// Map: Walkaeable and empty tiles on Row order
// MapDimensions: Size of Map - >= 1
// OutPath: The selected Path

bool FindPath(std::pair<int, int> Start, std::pair<int, int> Target, const std::vector<int>& Map, std::pair<int, int> MapDimensions, std::vector<int>& OutPath)
{
	int StartPoint = GetRawIndex(Start, MapDimensions.first);
	int EndPoint = GetRawIndex(Target, MapDimensions.first);

	std::vector<Node> OpenSet = std::vector<Node>();
	OpenSet.push_back(Node(StartPoint, 0, Distance(Start, Target, MapDimensions)));

	std::vector<Node> ClosedSet = std::vector<Node>();

	std::vector<Node> BestPath = std::vector<Node>();

	while (OpenSet.size() > 0)
	{
		size_t Current = GetCurrentNodeIndex(OpenSet);
		Node CurrentNode = OpenSet[Current];

		if (CurrentNode.RawIndex == EndPoint)
		{
			MakePath(BestPath, CurrentNode, OutPath);
			return true;
		}

		OpenSet.erase(OpenSet.begin() + Current);
		ClosedSet.push_back(CurrentNode);


	}

	return false;
}

void MakePath(const std::vector<Node>& NodeSet, const Node& current, std::vector<int>& OutPath)
{
	OutPath.push_back(current.RawIndex);
	size_t Size = NodeSet.size();
	for (size_t i = 0; i < Size; i++) OutPath.push_back(NodeSet[i].RawIndex);
}

size_t GetCurrentNodeIndex(const std::vector<Node>& NodeSet)
{
	size_t Current = 0;
	size_t Size = NodeSet.size();

	for (size_t i = 0; i < Size; i++)
	{
		if (NodeSet[i].FCost < NodeSet[Current].FCost)
			Current = i;
	}

	return Current;
}

//function reconstruct_path(cameFrom, current)
//total_path : = { current }
//while current in cameFrom.Keys :
//    current : = cameFrom[current]
//    total_path.prepend(current)
//
// return total_path
//
//    // A* finds a path from start to goal.
//    // h is the heuristic function. h(n) estimates the cost to reach goal from node n.
//function A_Star(start, goal, h)
//    // The set of discovered nodes that may need to be (re-)expanded.
//    // Initially, only the start node is known.
//    // This is usually implemented as a min-heap or priority queue rather than a hash-set.
//openSet: = { start }
//
//// For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start
//// to n currently known.
//cameFrom: = an empty map
//
//// For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
//gScore : = map with default value of Infinity
//gScore[start] : = 0
//
//// For node n, fScore[n] := gScore[n] + h(n). fScore[n] represents our current best guess as to
//// how cheap a path could be from start to finish if it goes through n.
//fScore: = map with default value of Infinity
//fScore[start] : = h(start)
//
//while openSet is not empty
//// This operation can occur in O(Log(N)) time if openSet is a min-heap or a priority queue
//current : = the node in openSet having the lowest fScore[] value
//if current = goal
//return reconstruct_path(cameFrom, current)
//
//openSet.Remove(current)
//for each neighbor of current
//// d(current,neighbor) is the weight of the edge from current to neighbor
//// tentative_gScore is the distance from start to the neighbor through current
//tentative_gScore : = gScore[current] + d(current, neighbor)
//if tentative_gScore < gScore[neighbor]
//    // This path to neighbor is better than any previous one. Record it!
//    cameFrom[neighbor] : = current
//    gScore[neighbor] : = tentative_gScore
//    fScore[neighbor] : = tentative_gScore + h(neighbor)
//    if neighbor not in openSet
//        openSet.add(neighbor)
//
//        // Open set is empty but goal was never reached
//        return failure