#pragma once
#include <utility>
#include <vector>

class FinderDebugger
{
public:
	FinderDebugger(std::pair<int, int> _StartPoint, std::pair<int, int> _EndPoint, std::vector<int> Map, std::pair<int, int> Dimensions);
	void DrawGrid(int drawTime);
	void SetCurrentPoint(std::pair<int, int> _CurrentPoint);
	void SetNeighbors(std::pair<int, int> Neighbor);
	void SetDefinedPath(std::pair<int, int> Start, const std::vector<int>& _DefinedPath);
	bool InPoint(std::pair<int, int> Point, std::pair<int, int> Target);

private:
	std::pair<int, int> StartPoint, EndPoint;
	std::pair<int, int> CurrentPoint;
	std::vector<std::pair<int, int>> Neighbors;
	std::vector<std::pair<int, int>> DefinedPath;

	std::pair<int, int> MapDimensions;
	std::vector<int> MapGrid;
};