import cv2
import metodosVision as mv
import os

# Load the image in BGR format
image_bgr = cv2.imread('D:\Dji\menosVision/foto.jpg', cv2.IMREAD_COLOR)
#cv2.imshow('Loaded RGB Image', image_bgr) 
#cv2.waitKey(0)  # Wait for a key press to close the window
# Check if image was loaded properly
#if image_bgr is not None:
#    # Convert the image from BGR to RGB
#    image_rgb = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2RGB)
#    print("Image loaded and converted to RGB.")
#else:
#    print("Error loading the image.")

bw, mask = mv.create_mask(image_bgr)
display_image = (bw.astype('float32') * 255).astype('uint8')



cv2.imshow('Loaded RGB Image', cv2.cvtColor(display_image, cv2.COLOR_RGB2BGR))  # Convert back to BGR for displaying
cv2.waitKey(0)  # Wait for a key press to close the window
cv2.destroyAllWindows()  # Close the window