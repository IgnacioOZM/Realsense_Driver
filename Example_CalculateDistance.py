import cv2
import numpy as np
from driver import realsense

def click_and_distance(event, x, y, flags, param):
       # grab references to the global variables
       global refPt, wait, mouse
       mouse = (x, y)
       if event == cv2.EVENT_LBUTTONDOWN:
              refPt.append((x,y))
       
       return

def CalculateDistance():
       global refPt, wait, camera

       # Adquisition of image
       images = camera.getImages(BRG=True, aling=True, depth_colour=True, colour=True, depth=True)
       colour = cv2.UMat(images['colour'])
       depth = cv2.UMat(images['depth'])

       # Draw a line between the two points of interest
       colour = cv2.line(colour, refPt[0], refPt[1], (0, 0, 255), 5)
       depth = cv2.line(depth, refPt[0], refPt[1], (0, 0, 255), 5)

       # Calculate the position of both points
       point1 = camera.getPosition(refPt[0])
       point2 = camera.getPosition(refPt[1])

       # Add point to image
       cord1 = "[" + "{:.2f}".format(point1[0]) + ", " + "{:.2f}".format(point1[1]) + ", "  + "{:.2f}".format(point1[2]) + "]"
       cord2 = "[" + "{:.2f}".format(point2[0]) + ", " + "{:.2f}".format(point2[1]) + ", "  + "{:.2f}".format(point2[2]) + "]"
       colour = cv2.putText(colour, cord1, (refPt[0][0],refPt[0][1]+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1, cv2.LINE_AA)
       colour = cv2.putText(colour, cord2, (refPt[1][0],refPt[1][1]-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1, cv2.LINE_AA)
       depth = cv2.putText(depth, cord1, (refPt[0][0],refPt[0][1]+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1, cv2.LINE_AA)
       depth = cv2.putText(depth, cord2, (refPt[1][0],refPt[1][1]-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1, cv2.LINE_AA)

       # Calculate distance module and display it
       distance = np.sqrt((point1[0]-point2[0])**2 +  (point1[1]-point2[1])**2 + (point1[2]-point2[2])**2)
       str = "Calculated distance: {:.2f} mm".format(distance)
       print(str)
       colour = cv2.putText(colour, str, (760,90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 3, cv2.LINE_AA)

       # Display new images
       cv2.imshow("Colour", colour)
       cv2.imshow("Depth", depth)
       wait = True
       return


if __name__ == "__main__":
       # Load camera module
       camera = realsense()
       # initialize with 0 points
       refPt = []
       wait = False
       mouse = (0,0)

       # Create and resize the Window
       cv2.destroyAllWindows()
       cv2.namedWindow('Colour', cv2.WINDOW_NORMAL)
       cv2.resizeWindow('Colour', 640, 480)
       cv2.namedWindow('Depth', cv2.WINDOW_NORMAL)
       cv2.resizeWindow('Depth', 640, 480)
       cv2.setMouseCallback("Colour", click_and_distance)

       while True:
              if wait == False:
                     # Adquisition of image
                     images = camera.getImages(BRG=True, aling=True, depth_colour=True, colour=True, depth=True)
                     colour = cv2.UMat(images['colour'])
                     depth = cv2.UMat(images['depth'])

                     # Transform from RGB to depth image
                     mouse_depth = mouse

                     # Addition of mouse coordinates
                     pos_col ="[" + "{:d}".format(mouse[0]) + ", " + "{:d}".format(mouse[1]) + "]"
                     pos_depth ="[" + "{:d}".format(mouse_depth[0]) + ", " + "{:d}".format(mouse_depth[1]) + "]"
                     colour = cv2.putText(colour, pos_col, (mouse[0], mouse[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)
                     colour = cv2.circle(colour, mouse, radius=1, color=(0,0,255), thickness=1)
                     depth = cv2.putText(depth, pos_depth, (mouse_depth[0], mouse_depth[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)
                     depth = cv2.circle(depth, mouse_depth, radius=1, color=(0,0,255), thickness=1)
                     
                     # Show images
                     cv2.imshow('Colour', colour)
                     cv2.imshow('Depth', depth)

              # Read keyboard
              key = cv2.waitKey(1) & 0xFF

              # Press Q or Esq on keyboard to exit
              if key == ord('q') or key == 27:
                     break

              # Resetting the system with Enter key
              if key == 13:
                     print('Resetting system')
                     refPt = []
                     wait = False
              
              # Press S on keyboard to save images
              if key == ord('s'):
                     # Saving aligned images
                     camera.saveImages(path = "Images/", aling = True, RGB = True, Depth = True, IR = True)
                     
              # if there are two reference points, then draw the line of interest
              # from both images, display it and calculate the distance
              if len(refPt) == 2 and wait == False:
                     CalculateDistance()

       # Close everuthing before exit
       cv2.destroyAllWindows()
       del camera