#include "PathFinder.h"
#include "FinderDebugger.h"
#include <stack>
#include <cmath>
#include <iostream>

// Start: Starting Point - >= 0 && < MapDimensions && IsWalkeable
// Target: Target Point - >= 0 && < MapDimensions && IsWalkeable
// Map: Walkaeable and empty tiles on Row order
// MapDimensions: Size of Map - >= 1
// OutPath: The selected Path

bool FindPath(std::pair<int, int> Start, std::pair<int, int> Target, const std::vector<int>& Map, std::pair<int, int> MapDimensions, std::vector<int>& OutPath)
{
	int StartPoint = GetRawIndex(Start, MapDimensions.second);
	int EndPoint = GetRawIndex(Target, MapDimensions.second);

	std::vector<Node> OpenSet = std::vector<Node>();
	std::vector<Node> ClosedSet = std::vector<Node>();
	std::vector<Node> BestPath = std::vector<Node>();
	std::vector<Node> Neighbors = std::vector<Node>();

	FinderDebugger Debugger = FinderDebugger(Start, Target, Map, MapDimensions);

	OpenSet.push_back(Node(StartPoint, 0, Distance(Start, Target, MapDimensions)));

	while (OpenSet.size() > 0)
	{
		size_t Current = GetCurrentNodeIndex(OpenSet);
		Node CurrentNode = OpenSet[Current];
		Debugger.SetCurrentPoint(GetCoords(CurrentNode.RawIndex, MapDimensions.first));

		if (CurrentNode.RawIndex == EndPoint)
		{
			MakePath(StartPoint, BestPath, CurrentNode, OutPath);
			Debugger.SetDefinedPath(Start, OutPath);
			Debugger.DrawGrid(5000);
			return true;
		}

		OpenSet.erase(OpenSet.begin() + Current);
		ClosedSet.push_back(CurrentNode);

		if (Neighbors.size()) Neighbors.clear();
		std::pair<int, int> Point = GetCoords(CurrentNode.RawIndex, MapDimensions.first);

		for (int i = 0, r = 0, c = 1; i < 4; i++, r = i >= 2, c = i < 2)
		{
			int sign = i == 0 || !(i % 2) ? 1 : -1;
			std::pair<int, int> NPoint = { Point.first + sign * c, Point.second + sign * r };
			if (!OnBounds(NPoint, MapDimensions)) continue;

			Debugger.SetNeighbors(NPoint);
			Node NeighborNode = Node(GetRawIndex(NPoint, MapDimensions.second),
				999,
				Distance(NPoint, Target, MapDimensions));
			Neighbors.push_back(NeighborNode);
		}

		size_t NSize = Neighbors.size();
		for (size_t i = 0; i < NSize; i++)
		{
			if (ContainsNode(ClosedSet, Neighbors[i])) continue;
			int GScore = CurrentNode.GCost + 1;/*Distance(GetCoords(Neighbors[i].RawIndex, MapDimensions.first), Point, MapDimensions)*/

			if (ContainsNode(OpenSet, Neighbors[i]))
			{
				if (GScore < Neighbors[i].GCost)
				{
					Neighbors[i].GCost = GScore;
					Neighbors[i].FCost = GScore + Neighbors[i].HCost;
				}
				continue;
			}

			if (!ContainsNode(BestPath, CurrentNode)) 
				BestPath.push_back(CurrentNode);

			Neighbors[i].GCost = GScore;
			Neighbors[i].FCost = GScore + Neighbors[i].HCost;
			OpenSet.insert(OpenSet.begin(), Neighbors[i]);
		}

		Debugger.DrawGrid(500);
	}

	return false;
}

int GetRawIndex(const std::pair<int, int>& Point, int SizeY) 
{ 
	return Point.first + (Point.second * (SizeY + 1)); 
}

std::pair<int, int> GetCoords(int Index, int SizeX)
{
	int y = Index / SizeX;
	return { Index - (y * SizeX), y };
}

int Distance(const std::pair<int, int>& Start, const std::pair<int, int>& End, const std::pair<int, int>& Dimensions)
{
	int x = abs(End.first - Start.first);
	int y = abs(End.second - Start.second);
	return x + y;
	//return sqrt((x * x) + (y * y));
}

bool OnBounds(const std::pair<int, int>& Point, const std::pair<int, int>& Dimensions)
{
	bool neg = (Point.first > -1) && (Point.second > -1);
	bool pos = (Point.first < Dimensions.first) && (Point.second < Dimensions.second);
	return neg && pos;
}

bool ContainsNode(const std::vector<Node>& NodeSet, const Node& NodeToCheck)
{
	size_t Size = NodeSet.size();
	for (size_t i = 0; i < Size; i++)
	{
		if (NodeSet[i].RawIndex == NodeToCheck.RawIndex)
			return true;
	}

	return false;
}

void MakePath(int startIndex, const std::vector<Node>& NodeSet, const Node& current, std::vector<int>& OutPath)
{
	size_t Size = NodeSet.size();
	for (size_t i = 0; i < Size; i++)
	{
		if (NodeSet[i].RawIndex == startIndex) continue;
		OutPath.push_back(NodeSet[i].RawIndex);
	}

	OutPath.push_back(current.RawIndex);
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

//void AddElementToOpenSet(const Node& NodeToAdd, const std::vector<Node>& OpenSet)
//{
//
//}

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