#pragma once

#include <Vector>
// A point in the map
struct Node
{
	int RawIndex, CellType;
	int FCost, GCost, HCost;

	// GCost: Distance from start to this node
	// HCost: Distance from the end to this node
	// FCost: The sum of G and H costs

	Node(int _RawIndex, int _CellType, int _GCost, int _HCost)
	{
		RawIndex = _RawIndex;
		CellType = _CellType;

		HCost = _HCost;
		UpdateNodeCost(_GCost);
	}

	void UpdateNodeCost(int _GCost) 
	{ 
		GCost = _GCost;
		FCost = GCost + HCost; 
	}
};

bool FindPath(std::pair<int, int> Start, std::pair<int, int> Target, const std::vector<int>& Map, std::pair<int, int> MapDimensions, std::vector<int>& OutPath);

int GetIndexFromCoords(const std::pair<int, int>& Point, int SizeY);
std::pair<int, int> GetCoordsFromIndex(int Index, int SizeX);

bool OnBounds(const std::pair<int, int>& Point, const std::pair<int, int>& Dimensions);
int Distance(const std::pair<int, int>& Start, const std::pair<int, int>& End, const std::pair<int, int>& Dimensions);
bool ContainsNode(const std::vector<Node>& NodeSet, const Node& NodeToCheck);
void BuildPath(int startIndex, const std::vector<Node>& NodeSet, const Node& current, std::vector<int>& OutPath);
size_t GetCurrentNodeIndex(const std::vector<Node>& NodeSet);
