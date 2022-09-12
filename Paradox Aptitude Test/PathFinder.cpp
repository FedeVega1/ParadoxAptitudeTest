#include "PathFinder.h"
#include "FinderDebugger.h"
#include <cmath>

bool FindPath(std::pair<int, int> Start, std::pair<int, int> Target, const std::vector<int>& Map, std::pair<int, int> MapDimensions, std::vector<int>& OutPath)
{
	if (Start.first == Target.first && Start.second == Target.second) return true;
	int StartIndex = GetIndexFromCoords(Start, MapDimensions.first);
	int EndIndex = GetIndexFromCoords(Target, MapDimensions.first);

	std::vector<Node> OpenSet = std::vector<Node>(); // Set of Nodes to analyze
	std::vector<Node> ClosedSet = std::vector<Node>(); // Set of Nodes already analyzed
	std::vector<Node> BestPath = std::vector<Node>(); // Best candidates
	std::vector<Node> CurrentNeighbours = std::vector<Node>(); // Neighbours of the Current analyzed Node

	FinderDebugger debug(Start, Target, Map, MapDimensions);
	debug.DrawGrid(1000);

	OpenSet.push_back(Node(StartIndex, Map[StartIndex], 0, Distance(Start, Target)));

	while (OpenSet.size() > 0)
	{
		// Get an open node with the lowest FCost.
		std::size_t Current = GetCurrentNodeIndex(OpenSet);
		Node CurrentNode = OpenSet[Current];
		debug.SetCurrentPoint(GetCoordsFromIndex(CurrentNode.RawIndex, MapDimensions.first));

		// If we reach our goal, build the path from the best nodes.
		if (CurrentNode.RawIndex == EndIndex)
		{
			BuildPath(StartIndex, BestPath, CurrentNode, OutPath);
			debug.SetDefinedPath(Start, OutPath);
			debug.DrawGrid(2000);
			return true;
		}

		// Lock current node.
		OpenSet.erase(OpenSet.begin() + Current);
		ClosedSet.push_back(CurrentNode);

		if (CurrentNeighbours.size() > 0) CurrentNeighbours.clear();
		std::pair<int, int> Point = GetCoordsFromIndex(CurrentNode.RawIndex, MapDimensions.first);

		// Get Neighbours from current Node.
		for (int i = 0; i < 4; i++)
		{
			// Get X-neighbours if i < 2 and get Y-neighbours if i >= 2.

			int sign = i == 0 || !(i % 2) ? 1 : -1;
			std::pair<int, int> NPoint = { Point.first + sign * (i < 2), Point.second + sign * (i >= 2)};
			if (!OnBounds(NPoint, MapDimensions)) continue;

			// If neighbour is not a traversable location, continue.
			int RawIndex = GetIndexFromCoords(NPoint, MapDimensions.first);
			if (!Map[RawIndex]) continue;

			debug.SetNeighbors(NPoint);
			CurrentNeighbours.push_back(Node(RawIndex, Map[RawIndex], 999, Distance(NPoint, Target)));
		}

		// Set Neighbours GCost and get candidates to analyze.
		std::size_t NSize = CurrentNeighbours.size();
		for (std::size_t i = 0; i < NSize; i++)
		{
			if (ContainsNode(ClosedSet, CurrentNeighbours[i])) continue;

			// The desired GScore is the current node GScore + the distance between nodes.
			int DesiredGScore = CurrentNode.GCost + 1;

			// If this neighbour is already on OpenSet, Update its cost
			if (ContainsNode(OpenSet, CurrentNeighbours[i]))
			{
				if (DesiredGScore < CurrentNeighbours[i].GCost)
					CurrentNeighbours[i].UpdateNodeCost(DesiredGScore);
				continue;
			}

			// Else push Current Node to candidates list and add the neighbour to the top of the list
			if (!ContainsNode(BestPath, CurrentNode)) 
				BestPath.push_back(CurrentNode);

			CurrentNeighbours[i].UpdateNodeCost(DesiredGScore);
			//OpenSet.push_back(CurrentNeighbours[i]);
			OpenSet.insert(OpenSet.begin(), CurrentNeighbours[i]);
		}

		debug.DrawGrid(1000);
	}

	return false;
}

int GetIndexFromCoords(const std::pair<int, int>& Point, int SizeX) { return Point.first + (Point.second * SizeX); }

std::pair<int, int> GetCoordsFromIndex(int Index, int SizeX)
{
	int y = Index / SizeX;
	return { Index - (y * SizeX), y };
}

int Distance(const std::pair<int, int>& Start, const std::pair<int, int>& End)
{
	int x = abs(End.first - Start.first);
	int y = abs(End.second - Start.second);
	return x + y;
}

bool OnBounds(const std::pair<int, int>& Point, const std::pair<int, int>& Dimensions)
{
	bool neg = (Point.first > -1) && (Point.second > -1);
	bool pos = (Point.first < Dimensions.first) && (Point.second < Dimensions.second);
	return neg && pos;
}

bool ContainsNode(const std::vector<Node>& NodeSet, const Node& NodeToCheck)
{
	std::size_t Size = NodeSet.size();
	for (std::size_t i = 0; i < Size; i++)
	{
		if (NodeSet[i].RawIndex == NodeToCheck.RawIndex)
			return true;
	}

	return false;
}

void BuildPath(int startIndex, const std::vector<Node>& NodeSet, const Node& current, std::vector<int>& OutPath)
{
	std::size_t Size = NodeSet.size();
	for (std::size_t i = 0, c = 0; i < Size; i++)
	{
		if (NodeSet[i].RawIndex == startIndex) continue;
		OutPath.push_back(NodeSet[i].RawIndex);
	}

	OutPath.push_back(current.RawIndex);
}

std::size_t GetCurrentNodeIndex(const std::vector<Node>& NodeSet)
{
	std::size_t Current = 0;
	std::size_t Size = NodeSet.size();

	for (std::size_t i = 0; i < Size; i++)
	{
		if (NodeSet[i].FCost < NodeSet[Current].FCost)
			Current = i;
	}

	return Current;
}
