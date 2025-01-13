import cv2
import numpy as np

class Maze:
    def __init__(self, image_path):
        # Load the maze image in grayscale
        self.maze_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if self.maze_image is None:
            raise ValueError("Error: Image not found.")
        
        # Threshold the image to convert it into a binary format
        _, self.binary_image = cv2.threshold(self.maze_image, 128, 255, cv2.THRESH_BINARY)
        self.height, self.width = self.binary_image.shape
        
        # Find start and goal positions
        self.start = self.find_start()
        self.goal = self.find_goal()
        
        # Create maze map
        self.maze_map = self.create_maze_map()
    
    def find_start(self):
        middle_x = self.width // 2
        for y in range(0, self.height):
            if self.binary_image[y, middle_x] == 255:
                return (middle_x, y)
        return None
    
    def find_goal(self):
        middle_x = self.width // 2
        for y in range(self.height - 1, -1, -1):
            if self.binary_image[y, middle_x] == 255:
                return (middle_x, y)
        return None
    
    def create_maze_map(self):
        maze_map = {}
        for x in range(self.width):
            for y in range(self.height):
                if self.binary_image[y, x] == 255:
                    directions = {'N': False, 'S': False, 'E': False, 'W': False}
                    # Check North
                    if y > 0 and self.binary_image[y - 1, x] == 255:
                        directions['N'] = True
                    # Check South
                    if y < self.height - 1 and self.binary_image[y + 1, x] == 255:
                        directions['S'] = True
                    # Check East
                    if x < self.width - 1 and self.binary_image[y, x + 1] == 255:
                        directions['E'] = True
                    # Check West
                    if x > 0 and self.binary_image[y, x - 1] == 255:
                        directions['W'] = True
                    maze_map[(x, y)] = directions
        return maze_map