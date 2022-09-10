#pragma once

#include <utility>
#include <Vector>

struct Node
{
	int RawIndex;
	int FCost, GCost, HCost;

	Node()
	{
		RawIndex = 0;
		FCost = 0;
		GCost = 0;
		HCost = 0;
	}

	Node(int _RawIndex, int _GCost, int _HCost)
	{
		RawIndex = _RawIndex;
		GCost = _GCost;
		HCost = _HCost;

		FCost = GCost + HCost;
	}
};

bool FindPath(std::pair<int, int> Start, std::pair<int, int> Target, const std::vector<int>& Map, std::pair<int, int> MapDimensions, std::vector<int>& OutPath);

int GetRawIndex(const std::pair<int, int>& Point, int SizeY);
std::pair<int, int> GetCoords(int Index, std::pair<int, int> Size);

bool OnBounds(const std::pair<int, int>& Point, const std::pair<int, int>& Dimensions);
int Distance(const std::pair<int, int>& Start, const std::pair<int, int>& End, const std::pair<int, int>& Dimensions);
bool ContainsNode(const std::vector<Node>& NodeSet, const Node& NodeToCheck);
void MakePath(const std::vector<Node>& NodeSet, const Node& current, std::vector<int>& OutPath);
size_t GetCurrentNodeIndex(const std::vector<Node>& NodeSet);
