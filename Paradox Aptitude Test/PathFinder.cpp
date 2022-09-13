#include "PathFinder.h"
#include "FinderDebugger.h"
#include <cmath>
#include <limits>

bool FindPath(std::pair<int, int> Start, std::pair<int, int> Target, const std::vector<int>& Map, std::pair<int, int> MapDimensions, std::vector<int>& OutPath)
{
	if (Start.first == Target.first && Start.second == Target.second) return true;
	int StartIndex = GetIndexFromCoords(Start, MapDimensions.first);
	int EndIndex = GetIndexFromCoords(Target, MapDimensions.first);

	std::vector<Node*> OpenSet; // Set of Nodes to analyze
	std::vector<Node*> CurrentNeighbours; // Neighbours of the current analyzed Node

	std::vector<Node> NodeMap;

	std::size_t TotalMapSize = Map.size();
	for (std::size_t i = 0; i < TotalMapSize; i++)
		NodeMap.push_back(Node(i, Map[i], std::numeric_limits<int>::max()));

	NodeMap[StartIndex].GCost = 0;
	NodeMap[StartIndex].HCost = Distance(Start, Target);
	OpenSet.push_back(&NodeMap[StartIndex]);

	/*FinderDebugger debug(Start, Target, Map, MapDimensions);
	debug.DrawGrid(1000);*/

	while (OpenSet.size() > 0)
	{
		// Get a node with the lowest FCost.
		std::size_t Current = GetCurrentNodeIndex(OpenSet);
		Node& CurrentNode = *OpenSet[Current];

		//debug.SetCurrentPoint(GetCoordsFromIndex(CurrentNode.RawIndex, MapDimensions.first));

		// If we reach our goal, build the path from the best nodes.
		if (CurrentNode.RawIndex == EndIndex)
		{
			BuildPath(NodeMap, StartIndex, CurrentNode, OutPath);
			//debug.SetDefinedPath(Start, OutPath);
			//debug.DrawGrid(2000);
			return true;
		}

		// Lock current node.
		OpenSet.erase(OpenSet.begin() + Current);
		CurrentNode.InList = true;

		if (CurrentNeighbours.size() > 0) CurrentNeighbours.clear();
		std::pair<int, int> Point = GetCoordsFromIndex(CurrentNode.RawIndex, MapDimensions.first);

		// Get Neighbours from current Node.
		for (int i = 0; i < 4; i++)
		{
			// Get X-neighbours if i < 2 and get Y-neighbours if i >= 2.

			int sign = i == 0 || !(i % 2) ? 1 : -1;
			std::pair<int, int> NPoint = { Point.first + sign * (i < 2), Point.second + sign * (i >= 2) };
			if (!OnBounds(NPoint, MapDimensions)) continue;

			// If neighbour is not a traversable location, continue.
			int RawIndex = GetIndexFromCoords(NPoint, MapDimensions.first);
			if (!Map[RawIndex]) continue;

			NodeMap[RawIndex].HCost = Distance(NPoint, Target);
			CurrentNeighbours.push_back(&NodeMap[RawIndex]);
		}

		// Set Neighbours GCost and get candidates to analyze.
		std::size_t NSize = CurrentNeighbours.size();
		for (std::size_t i = 0; i < NSize; i++)
		{
			if (CurrentNeighbours[i]->InList) continue;

			// The desired GScore is the current node GScore + the distance between nodes.
			int DesiredGScore = CurrentNode.GCost + 1;

			// If this neighbour is already on OpenSet, Update its cost
			if (ContainsNode(OpenSet, *CurrentNeighbours[i]))
			{
				if (DesiredGScore < CurrentNeighbours[i]->GCost)
					CurrentNeighbours[i]->UpdateNodeCost(DesiredGScore);
				continue;
			}

			CurrentNeighbours[i]->SetCameFromNode(CurrentNode.RawIndex);
			CurrentNeighbours[i]->UpdateNodeCost(DesiredGScore);
			OpenSet.push_back(CurrentNeighbours[i]);
		}

		//debug.DrawGrid(300);
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

bool ContainsNode(const std::vector<Node*>& NodeSet, const Node& NodeToCheck)
{
	std::size_t Size = NodeSet.size();
	for (std::size_t i = 0; i < Size; i++)
	{
		if (NodeSet[i]->RawIndex == NodeToCheck.RawIndex)
			return true;
	}

	return false;
}

void BuildPath(const std::vector<Node> NodeMap, int startIndex, const Node& current, std::vector<int>& OutPath)
{
	OutPath.insert(OutPath.begin(), current.RawIndex);
	int temp = current.CameFrom;

	while (temp != startIndex)
	{
		OutPath.insert(OutPath.begin(), temp);
		temp = NodeMap[temp].CameFrom;
	}
}

std::size_t GetCurrentNodeIndex(const std::vector<Node*>& NodeSet)
{
	std::size_t Current = 0;
	std::size_t Size = NodeSet.size();

	for (std::size_t i = 0; i < Size; i++)
	{
		if (NodeSet[i]->FCost < NodeSet[Current]->FCost)
			Current = i;
	}

	return Current;
}
