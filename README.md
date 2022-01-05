# Object-Recognition
This project focuses on using 3D projection of a scene obtained from Kinect. We can improve the accuracy of object recogintion by applying the depth information on top of SIFT.

We’ve seen SURF/SIFT features being used for matching objects from a database in a scene. Objects being matched always have a perspective transformation in the scene. 
RANSAC is known to work around these homographic transforms. But since the image is already distorted, the SIFT descriptors that come out of it are not the best for matching with descriptors extracted from a planar image in our database.
We can improve the accuracy and efficiency of matching by using depth information from a Kinect camera to remove or mitigate this perspective transformation. The Kinect camera will be able to provide us a 3D projection of a scene. We will first extract planar surfaces like walls, floors, ceilings and tables using
Hough transforms or RANSAC. We will then transform the image sections to their orthographic views.
