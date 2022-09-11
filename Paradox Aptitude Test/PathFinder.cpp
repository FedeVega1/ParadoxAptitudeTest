#include "PathFinder.h"
#include "FinderDebugger.h"
#include <stack>
#include <cmath>
#include <iostream>

bool FindPath(std::pair<int, int> Start, std::pair<int, int> Target, const std::vector<int>& Map, std::pair<int, int> MapDimensions, std::vector<int>& OutPath)
{
	int StartPoint = GetRawIndex(Start, MapDimensions.first);
	int EndPoint = GetRawIndex(Target, MapDimensions.first);

	std::vector<Node> OpenSet = std::vector<Node>();
	std::vector<Node> ClosedSet = std::vector<Node>();
	std::vector<Node> BestPath = std::vector<Node>();
	std::vector<Node> Neighbors = std::vector<Node>();

	FinderDebugger Debugger = FinderDebugger(Start, Target, Map, MapDimensions);
	Debugger.DrawGrid(1000);

	OpenSet.push_back(Node(StartPoint, Map[StartPoint], 0, Distance(Start, Target, MapDimensions)));

	while (OpenSet.size() > 0)
	{
		size_t Current = GetCurrentNodeIndex(OpenSet);
		Node CurrentNode = OpenSet[Current];
		Debugger.SetCurrentPoint(GetCoords(CurrentNode.RawIndex, MapDimensions.first));

		if (CurrentNode.RawIndex == EndPoint)
		{
			MakePath(StartPoint, BestPath, CurrentNode, OutPath);
			Debugger.SetDefinedPath(Start, OutPath);
			Debugger.DrawGrid(2000);
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

			int RawIndex = GetRawIndex(NPoint, MapDimensions.first);
			if (!Map[RawIndex]) continue;

			Node NeighborNode = Node(RawIndex, Map[RawIndex], 999, Distance(NPoint, Target, MapDimensions));

			Debugger.SetNeighbors(NPoint);
			Neighbors.push_back(NeighborNode);
		}

		size_t NSize = Neighbors.size();
		for (size_t i = 0; i < NSize; i++)
		{
			if (ContainsNode(ClosedSet, Neighbors[i])) continue;
			int GScore = CurrentNode.GCost + 1;

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

int GetRawIndex(const std::pair<int, int>& Point, int SizeX) { return Point.first + (Point.second * SizeX); }

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
