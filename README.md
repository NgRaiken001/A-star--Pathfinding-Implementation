# 🧭 A* Pathfinding Implementation in C++

An efficient and modular implementation of the **A\* (A-Star)** pathfinding algorithm using **C++**, designed for use in games and simulations requiring intelligent navigation over 2D grids. The algorithm finds the shortest path between two points while avoiding obstacles and minimizing traversal cost.

---

## 🎯 Objective

To implement a reusable A* algorithm in C++ that:
- Works on customizable grid-based maps
- Handles blocked/unwalkable nodes
- Outputs the shortest path using heuristics (Manhattan or Euclidean)

---

## 🧠 Features

- ✅ **A\* Algorithm Core Logic**
- 🧱 **Grid System with Obstacles**
- 🧮 **Heuristic Functions**: Manhattan and Euclidean supported
- 🔄 **Path Reconstruction**
- 📤 **Visual Output in Console** (for testing and debugging)
- 📦 **Modular & Extendable Codebase**

---

## 🛠️ Tech Stack

- **C++17**
- Standard Template Library (STL): `priority_queue`, `vector`, `unordered_map`
- Terminal-based output (no graphics dependency)

---

## 📁 Project Structure

```plaintext
/AStarPathfinding/
│
├── AStar.hpp/.cpp         # Core A* logic and pathfinding functions
├── Node.hpp               # Node structure representing grid tiles
├── Grid.hpp               # 2D grid management and utility
├── main.cpp               # Entry point with example usage
├── README.md
