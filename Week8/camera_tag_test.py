import cv2
import cv2.aruco as aruco
import numpy as np
import time
import math

# ==========================================
# CAMERA PARAMETERS
# These are approximate values for a webcam
# ==========================================

camera_matrix = np.array([
    [1000, 0, 640],
    [0, 1000, 360],
    [0, 0, 1]
], dtype=np.float32)

dist_coeffs = np.zeros((5, 1))

# Marker size in meters
MARKER_SIZE = 0.05  # 5 cm

# ==========================================
# OPEN WEBCAM
# ==========================================

cap = cv2.VideoCapture(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# ==========================================
# ARUCO SETUP
# ==========================================

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

parameters = aruco.DetectorParameters()

detector = aruco.ArucoDetector(
    aruco_dict,
    parameters
)

# ==========================================
# 3D CORNERS OF MARKER
# ==========================================

obj_points = np.array([
    [-MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
    [ MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
    [ MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
    [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0]
], dtype=np.float32)

# ==========================================
# FPS
# ==========================================

prev_time = time.time()

print("Press q to quit.")

# ==========================================
# MAIN LOOP
# ==========================================

while True:

    ret, frame = cap.read()

    if not ret:
        print("Failed to grab frame")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect markers
    corners, ids, rejected = detector.detectMarkers(gray)

    # ==========================================
    # FPS
    # ==========================================

    current_time = time.time()
    fps = 1.0 / (current_time - prev_time)
    prev_time = current_time
    print(f"FPS: {fps:.1f}")
    cv2.putText(
        frame,
        f"FPS: {fps:.1f}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2
    )

    # ==========================================
    # PROCESS MARKERS
    # ==========================================

    if ids is not None:

        aruco.drawDetectedMarkers(frame, corners, ids)

        for i in range(len(ids)):

            marker_id = ids[i][0]

            image_points = corners[i][0].astype(np.float32)

            # Pose estimation
            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                image_points,
                camera_matrix,
                dist_coeffs
            )

            if success:

                # Position
                x = tvec[0][0]
                y = tvec[1][0]
                z = tvec[2][0]

                # Draw coordinate axes
                cv2.drawFrameAxes(
                    frame,
                    camera_matrix,
                    dist_coeffs,
                    rvec,
                    tvec,
                    0.03
                )

                # Rotation matrix
                R, _ = cv2.Rodrigues(rvec)

                # Yaw angle
                yaw = math.degrees(
                    math.atan2(R[1, 0], R[0, 0])
                )

                # Marker center
                c = corners[i][0]

                center_x = int(c[:, 0].mean())
                center_y = int(c[:, 1].mean())

                # Text
                lines = [
                    f"ID: {marker_id}",
                    f"X: {x:.3f} m",
                    f"Y: {y:.3f} m",
                    f"Z: {z:.3f} m",
                    f"Yaw: {yaw:.1f} deg"
                ]

                for j, text in enumerate(lines):

                    cv2.putText(
                        frame,
                        text,
                        (center_x + 10, center_y + 25 * j),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )

    # ==========================================
    # SHOW WINDOW
    # ==========================================
    # Get image center
 # Get image center
    h, w, _ = frame.shape
    cx, cy = w // 2, h // 2

    color = (0, 0, 255)
    thickness = 2

    # vertical full line
    cv2.line(frame, (cx, 0), (cx, h), color, thickness)

    # horizontal full line
    cv2.line(frame, (0, cy), (w, cy), color, thickness)
    cv2.imshow("ArUco Pose Estimation", frame)

    # Quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ==========================================
# CLEANUP
# ==========================================

cap.release()
cv2.destroyAllWindows()