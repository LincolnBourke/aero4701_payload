import cv2 as cv

img = cv.imread("apps/camera/tests/image_close.png")

if img is None:
    print("None")

print(f"Original size: {img.shape[1]}x{img.shape[0]}")

# Crop: y1:y2, x1:x2
# x1, y1, x2, y2 = 140, 80, 500, 400 # good for camera min
# x1, y1, x2, y2 = 100, 50, 560, 430
# x1, y1, x2, y2 = 80, 40, 540, 440
# x1, y1, x2, y2 = 120, 40, 540, 450 # good for far
x1, y1, x2, y2 = 80, 20, 580, 480 # good for close (use for now)
cropped = img[y1:y2, x1:x2]

print(f"Cropped size:  {cropped.shape[1]}x{cropped.shape[0]}")

cv.imwrite("apps/camera/tests/cropped.png", cropped)

ratio = cropped.shape[1]*cropped.shape[0] / (640*480)

# ratio = 360*320 / (640*480)
print(ratio)