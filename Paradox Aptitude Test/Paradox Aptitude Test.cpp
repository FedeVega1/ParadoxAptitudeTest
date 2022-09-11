#include "PathFinder.h"
#include <iostream>
#include <chrono>

int main()
{
	std::vector<int> Map = { 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1 };
	std::vector<int> OutPath;

	//std::vector<int> Map = { 0, 0, 1, 0, 1, 1, 1, 0, 1 };
	//std::vector<int> OutPath;

	auto Start = std::chrono::high_resolution_clock::now();
	bool Result = FindPath({ 0, 0 }, { 1, 2 }, Map, { 4, 3 }, OutPath);
	//bool Result = FindPath({ 2, 0 }, { 0, 2 }, Map, { 3, 3 }, OutPath);
	auto End = std::chrono::high_resolution_clock::now();

	std::cout << "Total Time: " << std::chrono::duration<double, std::milli>(End - Start).count() << std::endl;

	if (Result)
	{
		std::cout << "Found path!" << std::endl;

		if (OutPath.size() > 0)
		{
			std::cout << "Path: { ";
			size_t Size = OutPath.size() - 1;
			for (size_t i = 0; i < Size; i++) std::cout << OutPath[i] << ", ";
			std::cout << OutPath[Size] << " }" << std::endl;
		}
		return 0;
	}

	std::cout << "No path found!" << std::endl;
	return 1;
}
