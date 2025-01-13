import cv2
import numpy as np

# Load the maze image
image_path = "/home/urvish/Documents/Path_Planning_with_Search_Algorithm/maze.png"  # Replace with the path to your maze image
maze_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

# Resize the image (optional, depends on the resolution of the maze)
resized_image = cv2.resize(maze_image, (39, 39), interpolation=cv2.INTER_NEAREST)
# You can adjust the dimensions based on your grid size.


# Threshold the image to convert it into a binary format
_, binary_image = cv2.threshold(maze_image, 128, 255, cv2.THRESH_BINARY)

# Convert binary image to a NumPy array where 0 = path and 1 = wall
maze_array = (binary_image == 0).astype(int)  # Walls are black (0 in binary_image)

# Print or save the array
print("Maze Array:")
print(maze_array)

# Save the array to a text file with the required formatting
with open("maze_array.txt", "w") as file:
    file.write("[\n")
    for row in maze_array:
        file.write(f"  {[int(val) for val in row]},\n")
    file.write("]\n")

# Visualize the binary image (optional)
cv2.imshow("Binary Maze", binary_image)
cv2.waitKey(0)
cv2.destroyAllWindows()