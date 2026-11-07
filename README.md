# 🤖 Maze-Solving Robot Simulator

A grid-based autonomous navigation simulator where a virtual robot navigates randomly generated mazes using classic pathfinding algorithms. Built with Python and pygame.

## 🧠 Algorithms Implemented
| Algorithm | Shortest Path | Strategy |
|-----------|--------------|----------|
| BFS | ✅ Yes | Explores level by level |
| DFS | ❌ No | Dives deep first |
| Dijkstra | ✅ Yes | Priority queue, uniform cost |
| A* | ✅ Yes | Heuristic-guided, fastest |

## 🎮 Controls
| Action | Control |
|--------|---------|
| New maze | Click **New Maze** |
| Run solver | Click **Solve** |
| Reset animation | Click **Reset View** |
| Change algorithm | Click BFS / DFS / Dijkstra / A* |
| Change speed | Click Slow / Medium / Fast |
| Quit | `ESC` |

## 🚀 Installation & Run

```bash
# Ubuntu
sudo apt install python3-pygame -y
python3 maze_simulator.py
```

## 📊 What to Observe
- 🔵 Blue cells = nodes explored by the robot
- 🟡 Yellow path = final optimal route
- 🔵 Cyan dot = robot moving in real time
- Stats panel shows nodes visited, path length, and solve time

## 💡 Key Insight
A* visits the fewest nodes because the Manhattan distance heuristic guides it toward the goal. BFS and Dijkstra explore blindly in all directions — you can literally see this difference in the animation.

## 🛠 Built With
- Python 3
- pygame
- Recursive backtracker maze generation
