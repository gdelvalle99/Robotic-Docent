import sys
import cv2 as cv
def main(argv):
    
    borderType = cv.BORDER_CONSTANT
    
    imageName = argv[0]
    # Loads an image
    src = cv.imread(imageName, cv.IMREAD_GRAYSCALE)
    
    
    top = int(0.005 * src.shape[0])  # shape[0] = rows
    bottom = top
    left = int(0.005 * src.shape[1])  # shape[1] = cols
    right = left

    value = 0
    
    dst = cv.copyMakeBorder(src, top, bottom, left, right, borderType, None, value)
    cv.imwrite(imageName, dst)
        
    return 0
if __name__ == "__main__":
    main(sys.argv[1:])