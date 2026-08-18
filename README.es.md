# Paradox Aptitude Test — Búsqueda de Caminos A* en C++

*[Read in English](README.md)*

Un proyecto pequeño y autocontenido en C++ que implementa el **algoritmo de búsqueda de caminos A\*** desde cero, construido como una prueba de aptitud técnica para una postulación laboral en Paradox Interactive. Busca la ruta más corta a través de un laberinto 2D grande y reporta cuánto tardó la búsqueda.

## Qué hace

El programa construye una **grilla de 50×960 celdas** (48.000 nodos) organizada como un laberinto alternado de paredes y pasillos abiertos, y luego busca el camino más corto de una esquina a la otra usando A*. Imprime:

- Si se encontró un camino
- La secuencia exacta de celdas que forman el camino
- El tiempo que tardó en calcularlo, medido en milisegundos

```
Total Time: 2.4351
Found path!
Path: { 0, 50, 100, 150, ... }
```

## Por qué es interesante

- **Algoritmo implementado desde cero** — sin librerías de pathfinding. Los conjuntos abierto/cerrado, el cálculo de costos (G/H/F) y la reconstrucción del camino están todos construidos a mano.
- **Con foco en rendimiento** — la búsqueda se mide con `std::chrono` con precisión de fracciones de milisegundo, y el tamaño de la grilla (48.000 celdas) es lo suficientemente grande como para que la eficiencia del algoritmo realmente importe.
- **Incluye un visualizador opcional** (`FinderDebugger`) que renderiza la búsqueda en vivo en la consola — mostrando la frontera de exploración, el nodo actual y el camino final como arte ASCII — útil para explicar *cómo* el algoritmo recorre la grilla, paso a paso.
- **Separación clara de responsabilidades**: el algoritmo (`PathFinder`), el visualizador (`FinderDebugger`) y el punto de entrada (`main`) están completamente desacoplados.

## Stack técnico

- **Lenguaje:** C++ (solo la Librería Estándar — sin dependencias externas)
- **Sistema de compilación:** Visual Studio 2022 / MSBuild (toolset `v143`)
- **Plataforma:** Windows (el visualizador de consola opcional usa la API de Windows; el algoritmo principal es STL portable)

## Cómo ejecutarlo

Abrí `Paradox Aptitude Test.sln` en Visual Studio y ejecutá, o compilá desde la línea de comandos:

```
msbuild "Paradox Aptitude Test.sln" /p:Configuration=Release /p:Platform=x64
```

---

*Compartido acá como muestra de habilidades de resolución de problemas algorítmicos y en C++.*
