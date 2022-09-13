#pragma once

#include <Vector>
#include <utility>

// A point in the map
struct Node
{
	int RawIndex, CellType;
	int FCost, GCost, HCost;
	bool InList;

	std::size_t CameFromIndex;

	// GCost: Distance from start to this node
	// HCost: Distance from the end to this node
	// FCost: The sum of G and H costs

	Node(int _RawIndex, int _CellType, int _GCost, int _HCost)
	{
		RawIndex = _RawIndex;
		CellType = _CellType;
		HCost = _HCost;
		InList = false;
		CameFromIndex = -1;
		UpdateNodeCost(_GCost);
	}

	Node(int _RawIndex, int _CellType, int _GCost)
	{
		RawIndex = _RawIndex;
		CellType = _CellType;
		HCost = 0;
		CameFromIndex = -1;
		UpdateNodeCost(_GCost);

		InList = false;
	}

	void UpdateNodeCost(int _GCost) 
	{ 
		GCost = _GCost;
		FCost = GCost + HCost; 
	}

	void SetCameFromNode(std::size_t _Index) { CameFromIndex = _Index; }
};

bool FindPath(std::pair<int, int> Start, std::pair<int, int> Target, const std::vector<int>& Map, std::pair<int, int> MapDimensions, std::vector<int>& OutPath);

int GetIndexFromCoords(const std::pair<int, int>& Point, int SizeY);
std::pair<int, int> GetCoordsFromIndex(int Index, int SizeX);

bool OnBounds(const std::pair<int, int>& Point, const std::pair<int, int>& Dimensions);
int Distance(const std::pair<int, int>& Start, const std::pair<int, int>& Endns);

bool ContainsNode(const std::vector<std::size_t>& IndexSet, std::size_t NodeIndexToCheck);
void BuildPath(const std::vector<Node>& NodeMap, int startIndex, std::size_t current, std::vector<int>& OutPath);
std::size_t GetCurrentNodeIndex(const std::vector<Node>& NodeMap, const std::vector<std::size_t>& OpenSet);
