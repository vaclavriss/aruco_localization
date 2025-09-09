import cv2
import cv2.aruco as aruco

# Vyber dictionary
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

# Vyber ID markeru (0 až 49 pro 4x4_50)
marker_id = 0
marker_size = 200  # px

# Vytvoř obrázek
img = aruco.drawMarker(aruco_dict, marker_id, marker_size)

# Ulož
cv2.imwrite("aruco_marker_id0.png", img)
print("Saved as aruco_marker_id0.png")
