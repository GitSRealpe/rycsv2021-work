import cv2
import numpy as np

img = cv2.imread("mapa_taller4.pgm", cv2.IMREAD_COLOR)
# img = cv2.imread("mapa_taller4.pgm",cv2.IMREAD_GRAYSCALE)

rotate=-8
img_width=img.shape[1]
img_heigth=img.shape[0]
M = cv2.getRotationMatrix2D((img_width/2,img_heigth/2), rotate, 1)
img = cv2.warpAffine(img, M, (img_width,img_heigth))

y=350
h=490
x=250
w=550
img = img[y:y+h, x:x+w]

GRID_SIZE = 20

height, width, channels = img.shape
# height, width = img.shape
for x in range(0, width -1, GRID_SIZE):
     cv2.line(img, (x, 0), (x, height), (255, 0, 0), 1, 1)


cv2.imshow("mapa", img)
cv2.waitKey(0)

cv2.imwrite("mapa_trans.pgm", img)

print("yolo")
cv2.destroyAllWindows()
