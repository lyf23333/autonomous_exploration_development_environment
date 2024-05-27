import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from PIL import Image
import os

def overlay_images_with_text(background_path, overlay_path, output_path, overlay_position, overlay_size, text, text_position, font_size):
    # Open the background and overlay images using PIL
    background = Image.open(background_path)
    overlay = Image.open(overlay_path)

    # Resize the overlay image
    overlay = overlay.resize(overlay_size)
    
    # Convert PIL images to arrays for Matplotlib
    background_array = mpimg.pil_to_array(background)
    overlay_array = mpimg.pil_to_array(overlay)

    # Create a figure and axis
    fig, ax = plt.subplots()

    # Display the background image
    ax.imshow(background_array)

    # Overlay the resized image
    ax.imshow(overlay_array, extent=(overlay_position[0], overlay_position[0] + overlay_size[0]/2, overlay_position[1], overlay_position[1] + overlay_size[1]/2))

    # Add text to the image
    ax.text(text_position[0], text_position[1], text, fontsize=font_size, color='white', bbox=dict(facecolor='black', alpha=0.5))

    # Remove the axes
    ax.axis('off')

    # Save the image
    plt.savefig(output_path, bbox_inches='tight', pad_inches=0)

# Example usage
home_dir = "/home/yifa/Pictures/rviz"
background_image_path = os.path.join(home_dir, 'rviz_indoor_LBPlanner3.png')
overlay_image_path = os.path.join(home_dir, 'rviz_indoor_FARPlanner1.png')
output_image_path = os.path.join(home_dir, 'rviz_indoor.png')
overlay_position = (653, 395)  # Position to place the overlay (x, y)
overlay_size = (230, 230)  # Size to resize the overlay image to (width, height)
text = 'Sample Text'
text_position = (50, 50)  # Position to place the text (x, y)
font_size = 24

overlay_images_with_text(background_image_path, overlay_image_path, output_image_path, overlay_position, overlay_size, text, text_position, font_size)
