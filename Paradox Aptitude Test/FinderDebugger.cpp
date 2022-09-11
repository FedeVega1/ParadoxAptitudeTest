#include "FinderDebugger.h"
#include <iostream>
#include <Windows.h>

FinderDebugger::FinderDebugger(std::pair<int, int> _StartPoint, std::pair<int, int> _EndPoint, std::vector<int> Map, std::pair<int, int> Dimensions)
{
	StartPoint = _StartPoint;
	EndPoint = _EndPoint;
	CurrentPoint = StartPoint;

	MapGrid = Map;
	MapDimensions = Dimensions;
}

void FinderDebugger::DrawGrid(int drawTime)
{
	system("CLS");

	for (size_t r = 0; r < MapDimensions.second; r++)
	{
		for (size_t c = 0; c < MapDimensions.first; c++)
		{
			std::pair<int, int> Point = { c, r };

			bool found = false;
			size_t PSize = DefinedPath.size();
			for (size_t i = 0; i < PSize; i++)
			{
				if (InPoint(Point, DefinedPath[i]))
				{
					std::cout << "[ # ]";
					found = true;
					break;
				}
			}

			if (found) continue;

			if (InPoint(Point, CurrentPoint))
			{
				std::cout << "[ X ]";
				continue;
			}

			found = false;
			size_t NSize = Neighbors.size();
			for (size_t i = 0; i < NSize; i++)
			{
				if (InPoint(Point, Neighbors[i]))
				{
					std::cout << "[ N ]";
					found = true;
					break;
				}
			}

			if (found) continue;

			if (InPoint(Point, StartPoint))
			{
				std::cout << "[ S ]";
				continue;
			}

			if (InPoint(Point, EndPoint))
			{
				std::cout << "[ E ]";
				continue;
			}

			std::cout << "[ " << MapGrid[c + (r * MapDimensions.first)] << " ]";
		}

		std::cout << std::endl;
	}

	if (Neighbors.size() > 0) Neighbors.clear();
	Sleep(drawTime);
}

void FinderDebugger::SetCurrentPoint(std::pair<int, int> _CurrentPoint) { CurrentPoint = _CurrentPoint; }
void FinderDebugger::SetNeighbors(std::pair<int, int> Neighbor) { Neighbors.push_back(Neighbor); }
bool FinderDebugger::InPoint(std::pair<int, int> Point, std::pair<int, int> Target) const { return Point.first == Target.first && Point.second == Target.second; }

void FinderDebugger::SetDefinedPath(std::pair<int, int> Start, const std::vector<int>& _DefinedPath)
{
	DefinedPath = std::vector<std::pair<int, int>>();
	DefinedPath.push_back(Start);

	size_t Size = _DefinedPath.size();
	for (size_t i = 0; i < Size; i++)
	{
		int y = _DefinedPath[i] / MapDimensions.first;
		DefinedPath.push_back({ _DefinedPath[i] - (y * MapDimensions.first), y });
	}
}