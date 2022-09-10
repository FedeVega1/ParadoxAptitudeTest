#include <iostream>
#include "PathFinder.h"

int main()
{
	std::vector<int> Map = { 1, 1, 1, 1, 
							 0, 1, 0, 1, 
							 0, 1, 1, 1 };
	std::vector<int> OutPath;

	bool result = FindPath({ 0, 0 }, { 1, 2 }, Map, { 4, 3 }, OutPath);
	if (result) 
	{
		std::cout << "Found path!" << std::endl;
		std::cout << "Path: { ";

		size_t size = OutPath.size() - 1;
		for (size_t i = 0; i < size; i++) std::cout << OutPath[i] << ", ";
		std::cout << OutPath[size] << " }" << std::endl;
		return 0;
	}

	std::cout << "No path found!" << std::endl;
	return 1;
}
