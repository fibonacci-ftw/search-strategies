# search-strategies
## Implementation of BFS, DFS and A* Search in Python.  
Given an elevation matrix, threshold for traversing elevations, finds a path from source to destination using either BFS, DFS or A*.  
### Input format:    
Name of Search: BFS, DFS or A*.  
Width(W) and Height(H) of Elevation Matrix.  
Threshold of elevation difference you are allowed to traverse.  
Number of destination positions.  
x,y coordinate of each destination separated by a new line.  
H lines with elevation values for W positions in the matrix.

#### Eg:  
BFS   
2 2   
0 0   
5   
1  
1 1  
0 10   
10 20.   

### Output Format:
Path from source to destination. New line for each destination, each coordinate position separated by a space and x,y separated by a comma. FAIL if no such path exists.

#### Eg:   
4,4 5,4 6,3  
4,4 3,4 2,3 2,2 1,1   
FAIL

**_input_generator.py:_**
Generates random inputs in the required format for testing.
