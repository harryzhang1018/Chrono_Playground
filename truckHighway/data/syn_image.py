import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load the car image
car_image_path = "/home/harry/render_output.png"
car_img = cv2.imread(car_image_path, cv2.IMREAD_UNCHANGED)

# Ensure the image has an alpha channel (transparency)
if car_img.shape[-1] == 3:  # Convert to RGBA if needed
    car_img = cv2.cvtColor(car_img, cv2.COLOR_BGR2BGRA)

# Resize the car image to a smaller size
car_img = cv2.resize(car_img, (100, 50), interpolation=cv2.INTER_AREA)

# Define canvas size
canvas_width, canvas_height = 1000, 500
canvas = np.zeros((canvas_height, canvas_width, 4), dtype=np.uint8)

# Define trajectory
num_cars = 20
x_positions = np.linspace(100, 900, num_cars)
y_positions = np.concatenate((
    np.full(num_cars//4, 250),  # First straight
    250 + 50 * np.cos(np.linspace(0, np.pi, num_cars//2)),  # Double lane change
    np.full(num_cars//4, 250)  # Second straight
))

# Compute heading angles as the tangent of the trajectory
dx = np.gradient(x_positions)
dy = np.gradient(y_positions)
angles = np.arctan2(dy, dx) * 180 / np.pi  # Convert to degrees

# Function to overlay a rotated car image onto the canvas
def overlay_image(canvas, img, x, y, angle):
    # Get image dimensions
    h, w = img.shape[:2]
    
    # Rotate image
    center = (w//2, h//2)
    rot_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated_img = cv2.warpAffine(img, rot_matrix, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0, 0))
    
    # Calculate position for placing the image
    x, y = int(x - w//2), int(y - h//2)
    
    # Ensure the car image fits within the canvas bounds
    if y < 0 or x < 0 or y + h > canvas.shape[0] or x + w > canvas.shape[1]:
        return canvas
    
    # Overlay image with transparency
    for c in range(3):  # BGR channels
        mask = rotated_img[:, :, 3] / 255.0  # Alpha channel as mask
        canvas[y:y+h, x:x+w, c] = (1 - mask) * canvas[y:y+h, x:x+w, c] + mask * rotated_img[:, :, c]
    
    return canvas

# Place cars on the canvas
for i in range(num_cars):
    canvas = overlay_image(canvas, car_img, x_positions[i], y_positions[i], angles[i])

# Convert to RGB and display
canvas_rgb = cv2.cvtColor(canvas, cv2.COLOR_BGRA2RGBA)
plt.figure(figsize=(10, 5))
plt.imshow(canvas_rgb)
plt.axis("off")
plt.show()

# Save the output image
output_path = "/home/harry/synthesized_trajectory.png"
cv2.imwrite(output_path, cv2.cvtColor(canvas, cv2.COLOR_BGRA2RGBA))
print(f"Synthesized image saved at {output_path}")
