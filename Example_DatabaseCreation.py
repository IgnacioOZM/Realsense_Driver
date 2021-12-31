import cv2
import os
from PIL import Image
from driver import realsense

def save_images(path: str, image):
    # Check directory
    os.makedirs(path, exist_ok=True)

    # Check image counter
    pngCounter = 0
    top = os.getcwd() + "/"
    for file in os.listdir(top + path):
            if file.endswith(".png"):
                pngCounter += 1
    print("Saving images on:" + top + path)
    print("Current number of pictures: {}".format(pngCounter+1))

    # Adquisition of new and clean images with alingment
    images = camera.getImages(BRG = True, colour = True)
    colour = images['colour']

    # Save images
    colour_str = top + path + "{:04d}".format(pngCounter) + ".png"
    cv2.imwrite(colour_str, colour)
    return

if __name__ == "__main__":
    # Load camera module
    camera = realsense()

    # Parameters
    original = True
    alpha = 0.5
    
    # Create and resize the Window
    cv2.destroyAllWindows()
    cv2.namedWindow('Colour', cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty('Colour', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        
    while True:
        # Adquisition of image
        images = camera.getImages(BRG=True, colour=True)
        colour = images['colour']

        # Display images
        if original == True:
            cv2.imshow('Colour', colour)
        else:
            
            added_image = cv2.addWeighted(colour,alpha,original_image,1-alpha,0)
            cv2.imshow('Colour', added_image)
        
        # Read keyboard
        key = cv2.waitKey(1) & 0xFF

        # Press S on keyboard to save images
        if key == ord('s'):
            # Savig of original image
            if original == True:
                original = False
                save_path = "Database/original/"
                original_image = colour
            else:
                original = True
                save_path = "Database/Aruco/"
            
            # Saving original image
            save_images(path = save_path, image = colour)

        # Press Q or Esq on keyboard to exit
        if key == ord('q') or key == 27:
                break

        # Resetting the system with Enter key
        if key == 13:
                print('Resetting system')
                refPt = []
                original = True

    # Close everything before exit
    del camera
    cv2.destroyAllWindows()