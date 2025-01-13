import cv2
import numpy as np
from queue import PriorityQueue
from maze import Maze
import argparse

def heuristics(cell1, cell2):
    x1, y1 = cell1
    x2, y2 = cell2
    return abs(x1 - x2) + abs(y1 - y2)

def astar(maze):
    start = maze.start
    goal = maze.goal
    maze_map = maze.maze_map
    
    g_score = {cell: float('inf') for cell in maze_map}
    f_score = {cell: float('inf') for cell in maze_map}
    g_score[start] = 0
    f_score[start] = heuristics(start, goal)
    
    open_set = PriorityQueue()
    open_set.put((f_score[start], heuristics(start, goal), start))
    
    came_from = {}
    
    while not open_set.empty():
        current_f, h, current = open_set.get()
        
        if current == goal:
            break
        
        for direction, is_open in maze_map[current].items():
            if is_open:
                if direction == 'N':
                    neighbor = (current[0], current[1] - 1)
                elif direction == 'S':
                    neighbor = (current[0], current[1] + 1)
                elif direction == 'E':
                    neighbor = (current[0] + 1, current[1])
                elif direction == 'W':
                    neighbor = (current[0] - 1, current[1])
                
                if 0 <= neighbor[0] < maze.width and 0 <= neighbor[1] < maze.height:
                    temp_g = g_score[current] + 1
                    temp_f = temp_g + heuristics(neighbor, goal)
                    if temp_f < f_score.get(neighbor, float('inf')):
                        g_score[neighbor] = temp_g
                        f_score[neighbor] = temp_f
                        open_set.put((temp_f, heuristics(neighbor, goal), neighbor))
                        came_from[neighbor] = current

    path = reconstruct_path(came_from, start, goal)
    return path

def reconstruct_path(came_from, start, goal):
    path = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from.get(current, None)
        if current is None:
            break
    path.append(start)
    return path[::-1]

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Solve a maze using A* algorithm.")
    parser.add_argument("image_path", type=str, help="Path to the maze image file.")
    args = parser.parse_args()
    
    maze = Maze(args.image_path)
    path = astar(maze)
    
    # Visualize the path on the original image
    color_image = cv2.cvtColor(maze.binary_image, cv2.COLOR_GRAY2BGR)
    
    # Set the line thickness dynamically based on the maze resolution
    line_thickness = max(1, min(maze.width, maze.height) // 100)
    
    for i in range(len(path) - 1):
        p1 = (int(path[i][0] + 0.5), int(path[i][1] + 0.5))
        p2 = (int(path[i + 1][0] + 0.5), int(path[i + 1][1] + 0.5))
        cv2.line(color_image, p1, p2, (0, 255, 0), thickness=line_thickness)
    
    # Draw start and goal markers
    if maze.start:
        start_centered = (int(maze.start[0] + 0.5), int(maze.start[1] + 0.5))
        cv2.circle(color_image, start_centered, line_thickness + 5, (0, 255, 0), -1)
    
    if maze.goal:
        goal_centered = (int(maze.goal[0] + 0.5), int(maze.goal[1] + 0.5))
        cv2.circle(color_image, goal_centered, line_thickness + 5, (0, 0, 255), -1)
    
    # Animation parameters
    marker_size = 5
    marker_color = (0, 0, 255)  # Red
    delay = 50  # milliseconds
    
    # Create a base image with the path
    base_image = color_image.copy()
    
    # Initialize VideoWriter to save the animation as a video
    frame_width = base_image.shape[1]
    frame_height = base_image.shape[0]
    fourcc = cv2.VideoWriter_fourcc(*'XVID')  # You can change the codec (e.g., 'MP4V' for MP4)
    out = cv2.VideoWriter('outputvideo.avi', fourcc, 20.0, (frame_width, frame_height))
    
    # Animate the marker moving along the path
    for point in path:
        # Create a copy of the base image for each frame
        current_image = base_image.copy()
        
        # Draw the marker at the current position
        cv2.circle(current_image, (int(point[0] + 0.5), int(point[1] + 0.5)), marker_size, marker_color, -1)
        
        # Write the frame to the video
        out.write(current_image)
        
        # Display the image
        cv2.imshow("Maze with Moving Marker", current_image)
        
        # Introduce a delay and check for user input to exit
        key = cv2.waitKey(delay)
        if key == ord('q'):  # Exit if 'q' is pressed
            break
    
    # Release the video writer
    out.release()
    
    cv2.destroyAllWindows()
