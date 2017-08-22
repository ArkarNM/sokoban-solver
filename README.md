# Sokoban Puzzle Solver
An assignment for CSC384. Note to UofT students, please **do not plagarize**. MarkUs ***will*** catch you.  
---

### Description
Sokoban is a puzzle game in which a warehouse robot must push boxes into storage spaces. The rules hold that only one box can be moved at a time, that boxes can only be pushed by the robot and not pulled, and that neither robots nor boxes can pass through obstacles (walls or other boxes). In addition, the robot cannot push more than one box, i.e., if there are two boxes in a row, the robot cannot push them. The game is over when all the boxes are in their storage spots. In our version of Sokoban, the rules are slightly more complicated, as there may be restrictions on which storage spaces are allowed for each box.

The functions implemented include: 
* Manhattan distance heuristic 
* Anytime Greedy Best-First Search
* Implement Anytime Weighted A*
* A non-trivial heuristic that improves on the Manhattan distance heuristic