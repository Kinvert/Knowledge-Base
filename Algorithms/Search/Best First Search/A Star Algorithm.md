## Overview

A* (pronounced "A-star") is a widely used graph traversal and pathfinding algorithm in computer science. It is known for its efficiency and optimality in finding the shortest path between a start node and a goal node in a weighted graph. A* is commonly used in applications such as robotics, game development, and artificial intelligence[2](https://en.wikipedia.org/wiki/A*_search_algorithm)[3](https://www.geeksforgeeks.org/a-search-algorithm/)[8](https://brilliant.org/wiki/a-star-search/).

## Key Concepts

- **Informed Search**: A* is an informed (best-first) search algorithm, meaning it uses additional information (a heuristic) to guide its search and prioritize which nodes to explore next[1](https://www.datacamp.com/tutorial/a-star-algorithm)[2](https://en.wikipedia.org/wiki/A*_search_algorithm)[8](https://brilliant.org/wiki/a-star-search/).
    
- **Heuristic Function (ℎ(�)h(n))**: Estimates the cost from a given node �n to the goal. The choice of heuristic greatly affects the algorithm’s performance and guarantees optimality if it is admissible (never overestimates the true cost)[1](https://www.datacamp.com/tutorial/a-star-algorithm)[2](https://en.wikipedia.org/wiki/A*_search_algorithm)[8](https://brilliant.org/wiki/a-star-search/).
    
- **Cost Function (�(�)g(n))**: The actual cost from the start node to node �n[1](https://www.datacamp.com/tutorial/a-star-algorithm)[2](https://en.wikipedia.org/wiki/A*_search_algorithm)[8](https://brilliant.org/wiki/a-star-search/).
    
- **Evaluation Function (�(�)f(n))**: The sum of the actual cost and the heuristic estimate:
    
    �(�)=�(�)+ℎ(�)f(n)=g(n)+h(n)
    
    This function determines which node to expand next[2](https://en.wikipedia.org/wiki/A*_search_algorithm)[8](https://brilliant.org/wiki/a-star-search/).
    

## How A* Works

## Initialization

- **Open List**: Contains nodes to be evaluated (starts with the start node).
    
- **Closed List**: Contains nodes already evaluated (starts empty)[1](https://www.datacamp.com/tutorial/a-star-algorithm).
    

Each node stores:

- �g: Cost from start node
    
- ℎh: Heuristic estimate to goal
    
- �f: Total estimated cost (�+ℎg+h)
    
- **Parent**: Reference for path reconstruction[1](https://www.datacamp.com/tutorial/a-star-algorithm).
    

## Main Loop

1. Select the node from the open list with the lowest �f value.
    
2. Move it to the closed list.
    
3. For each neighbor of the current node:
    
    - Skip if already in the closed list.
        
    - Calculate tentative �g score.
        
    - If this path to the neighbor is better, update its �g, ℎh, �f, and parent.
        
    - Add neighbor to the open list if not already present[1](https://www.datacamp.com/tutorial/a-star-algorithm)[2](https://en.wikipedia.org/wiki/A*_search_algorithm)[8](https://brilliant.org/wiki/a-star-search/).
        
4. Repeat until the goal node is reached or the open list is empty.
    

## Path Reconstruction

- Once the goal is reached, backtrack using parent references to reconstruct the optimal path from start to goal[1](https://www.datacamp.com/tutorial/a-star-algorithm).
    

## Properties

- **Optimality**: Finds the shortest path if the heuristic is admissible[1](https://www.datacamp.com/tutorial/a-star-algorithm)[2](https://en.wikipedia.org/wiki/A*_search_algorithm)[8](https://brilliant.org/wiki/a-star-search/).
    
- **Completeness**: Will find a path if one exists[2](https://en.wikipedia.org/wiki/A*_search_algorithm)[3](https://www.geeksforgeeks.org/a-search-algorithm/).
    
- **Efficiency**: Outperforms uninformed search algorithms (like Dijkstra’s) by using the heuristic to focus the search[8](https://brilliant.org/wiki/a-star-search/).
    

## Pseudocode

1. **Initialize**
    - Let **openList** be a priority queue containing the start node.
    - Set `g(start) = 0`.
    - Set `h(start) = heuristic estimate from start to goal`.
    - Set `f(start) = g(start) + h(start)`.
    - Let **closedList** be an empty set.
2. **Main Loop**  
    While **openList** is not empty:
    1. **Current Node Selection**
        - Remove the node `current` from **openList** with the lowest `f(current)`.
    2. **Goal Test**
        - If `current` is the goal node:
            - Reconstruct the path from start to goal by following parent links.
            - Return the path.
    3. **Closed List Update**
        - Add `current` to **closedList**.
    4. **Neighbor Exploration**  
        For each neighbor `neighbor` of `current`:
        1. **Skip if Closed**
            - If `neighbor` is in **closedList**, continue to next neighbor.
        2. **Tentative g Calculation**
            - Compute `tentative_g = g(current) + cost(current, neighbor)`.
        3. **Open List and Score Update**
            - If `neighbor` is not in **openList** OR `tentative_g < g(neighbor)`:
                - Set `parent(neighbor) = current`.
                - Set `g(neighbor) = tentative_g`.
                - Set `h(neighbor) = heuristic estimate from neighbor to goal`.
                - Set `f(neighbor) = g(neighbor) + h(neighbor)`.
                - If `neighbor` is not in **openList**, add it.
3. **Failure Condition**
    - If **openList** is empty and goal was not reached, return failure (no path found).

**Notes**

- **g(n):** Cost from start to node n.
    
- **h(n):** Heuristic estimate from node n to goal.
    
- **f(n):** Total estimated cost of path through n (`f(n) = g(n) + h(n)`).
---
## Applications

- **Game AI**: Pathfinding for non-player characters.
    
- **Robotics**: Navigation in physical space.
    
- **Map Routing**: GPS and logistics optimization[4](https://www.simplilearn.com/tutorials/artificial-intelligence-tutorial/a-star-algorithm)5[8](https://brilliant.org/wiki/a-star-search/).


> "A* is an extension of Dijkstra's algorithm with some characteristics of breadth-first search (BFS). What makes A* different is its use of a heuristic function to estimate the total cost of a path through each node, allowing it to focus the search and find optimal solutions efficiently."

## Links

https://github.com/Kinvert/CUDA-Stuff/tree/master/Algorithms/Search/Best-First-Search/A-Star-Algorithm

![datacamp.com favicon](https://www.google.com/s2/favicons?sz=128&domain=datacamp.com)

1 · DataCamp

datacamp.com/tutorial/a-sta…

The A* Algorithm: A Complete Guide | DataCamp

A guide to understanding and implementing the A* search algorithm in Python.

![en.wikipedia.org favicon](https://www.google.com/s2/favicons?sz=128&domain=en.wikipedia.org)

2 · Wikimedia Foundation, Inc.

en.wikipedia.org/wiki/A*_search…

A* search algorithm - Wikipedia

A* (pronounced "A-star") is a graph traversal and pathfinding algorithm that is used in many fields of computer science due to its completeness, optimality, ...

![geeksforgeeks.org favicon](https://www.google.com/s2/favicons?sz=128&domain=geeksforgeeks.org)

3 · geeksforgeeks

geeksforgeeks.org/a-search-algor…

A* Search Algorithm - GeeksforGeeks

A* Search algorithm is one of the best and popular technique used in path-finding and graph traversals.

![simplilearn.com favicon](https://www.google.com/s2/favicons?sz=128&domain=simplilearn.com)

4 · simplilearn

simplilearn.com/tutorials/arti…

A* Algorithm: A Comprehensive Guide - Simplilearn.com

A* Search Algorithm is a simple and efficient search algorithm that can be used to find the optimal path between two nodes in a graph.

![youtube.com favicon](https://www.google.com/s2/favicons?sz=128&domain=youtube.com)

5 · youtube

youtube.com/watch?v=kEY1Ox…

A* algorithm EASY explained (example) - YouTube

![gist.github.com favicon](https://www.google.com/s2/favicons?sz=128&domain=gist.github.com)

6 · Gist

gist.github.com/saharshbhansal…

Markdown Formatting Guide for Obsidian - GitHub Gist


![brilliant.org favicon](https://www.google.com/s2/favicons?sz=128&domain=brilliant.org)

8 · Brilliantorg

brilliant.org/wiki/a-star-se…

A* Search | Brilliant Math & Science Wiki

A* (pronounced as "A star") is a computer algorithm that is widely used in pathfinding and graph traversal. The algorithm efficiently plots a walkable path between multiple nodes, or points, on the graph. On a map with many obstacles, pathfinding from points ...
