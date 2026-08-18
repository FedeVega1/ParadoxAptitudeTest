# Paradox Aptitude Test — A* Pathfinding in C++

*[Leer en español](README.es.md)*

A small, self-contained C++ project implementing the **A\* pathfinding algorithm** from scratch, built as a coding aptitude test for a job application to Paradox Interactive. It searches for the shortest route across a large 2D grid maze and reports how long the search took.

## What it does

The program builds a **50×960 cell grid** (48,000 nodes) laid out as an alternating maze of walls and open corridors, then searches for the shortest path from one corner to the other using A*. It prints:

- Whether a path was found
- The exact sequence of cells that make up the path
- The time taken to compute it, measured in milliseconds

```
Total Time: 2.4351
Found path!
Path: { 0, 50, 100, 150, ... }
```

## Why it's interesting

- **Algorithm implemented from first principles** — no pathfinding libraries. Open/closed sets, cost tracking (G/H/F costs), and path reconstruction are all hand-built.
- **Performance-aware** — the search is timed with `std::chrono` down to fractions of a millisecond, and the grid size (48,000 cells) is large enough to make algorithmic efficiency actually matter.
- **Includes an optional visual debugger** (`FinderDebugger`) that renders the search live in the console — showing the frontier, current node, and final path as ASCII art — useful for explaining *how* the algorithm explores the grid, step by step.
- **Clean separation of concerns**: the algorithm (`PathFinder`), the visualizer (`FinderDebugger`), and the entry point (`main`) are fully decoupled.

## Tech stack

- **Language:** C++ (Standard Library only — no external dependencies)
- **Build system:** Visual Studio 2022 / MSBuild (`v143` toolset)
- **Platform:** Windows (the optional console visualizer uses the Windows API; the core algorithm is portable STL)

## Running it

Open `Paradox Aptitude Test.sln` in Visual Studio and run, or build from the command line:

```
msbuild "Paradox Aptitude Test.sln" /p:Configuration=Release /p:Platform=x64
```

---

*Shared here as a sample of algorithmic and C++ problem-solving skills.*
