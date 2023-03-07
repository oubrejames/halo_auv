# from auv_move import HaloAUV
import apriltag
import argparse
import cv2
from camera_stream import Video

# Start stream
video = Video()
# video.start_stream()
print("HEHEHEHEH")
# Get latest frame

while True:
    # Wait for the next frame to become available
    if video.frame_available():
        frame = video.frame()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        print("[INFO] detecting AprilTags...")
        options = apriltag.DetectorOptions(families="tag36h11")
        detector = apriltag.Detector(options)
        results = detector.detect(gray)
        print("[INFO] {} total AprilTags detected".format(len(results)))
        # Only retrieve and display a frame if it's new
        cv2.imshow('frame', frame)
        

# Detect apriltags



# loop over the AprilTag detection results
for r in results:
	# extract the bounding box (x, y)-coordinates for the AprilTag
	# and convert each of the (x, y)-coordinate pairs to integers
	(ptA, ptB, ptC, ptD) = r.corners
	ptB = (int(ptB[0]), int(ptB[1]))
	ptC = (int(ptC[0]), int(ptC[1]))
	ptD = (int(ptD[0]), int(ptD[1]))
	ptA = (int(ptA[0]), int(ptA[1]))
	# draw the bounding box of the AprilTag detection
	cv2.line(image, ptA, ptB, (0, 255, 0), 2)
	cv2.line(image, ptB, ptC, (0, 255, 0), 2)
	cv2.line(image, ptC, ptD, (0, 255, 0), 2)
	cv2.line(image, ptD, ptA, (0, 255, 0), 2)
	# draw the center (x, y)-coordinates of the AprilTag
	(cX, cY) = (int(r.center[0]), int(r.center[1]))
	cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
	# draw the tag family on the image
	tagFamily = r.tag_family.decode("utf-8")
	cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
		cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
	print("[INFO] tag family: {}".format(tagFamily))
# show the output image after AprilTag detection
cv2.imshow("Image", image)
cv2.waitKey(0)


# auv = HaloAUV()
# # auv.restart_pixhawk()
# auv.arm()
# auv.set_relative_depth(250.35)
# auv.hold_depth()
# # while(1):
# #     auv.ascend(600)