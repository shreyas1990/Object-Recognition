# Object-Recognition
This project focuses on using 3D projection of a scene obtained from Kinect. We can improve the accuracy of object recogintion by applying the depth information on top of SIFT.

We’ve seen SURF/SIFT features being used for matching objects from a database in a scene. Objects being matched always have a perspective transformation in the scene. 
RANSAC is known to work around these homographic transforms. But since the image is already distorted, the SIFT descriptors that come out of it are not the best for matching with descriptors extracted from a planar image in our database.
We can improve the accuracy and efficiency of matching by using depth information from a Kinect camera to remove or mitigate this perspective transformation. The Kinect camera will be able to provide us a 3D projection of a scene. We will first extract planar surfaces like walls, floors, ceilings and tables using
Hough transforms or RANSAC. We will then transform the image sections to their orthographic views.

WORKING STEPS
● Use EGT to generate simulated data.
![1](https://user-images.githubusercontent.com/29533107/148198845-542388b7-19da-49e2-90c2-5abc8ab7b4d1.jpg) 

The 3D scene will be seen in Kinect as shown below:
![2](https://user-images.githubusercontent.com/29533107/148199228-21049d3c-8606-4ed0-9c1a-dadecf55dd52.jpg)

● Segment planes using the simulated data to get planes.
![5](https://user-images.githubusercontent.com/29533107/148199704-f922ea87-3029-4ac9-ba54-57fde48dcf8a.jpg)
![7](https://user-images.githubusercontent.com/29533107/148199745-3ef8c130-bb22-4c85-88e0-0385c2a8e427.jpg)
![9](https://user-images.githubusercontent.com/29533107/148199785-89d81ee2-a1ba-4615-b9da-9c7ddeeda209.jpg)

● Undistort the planes using homography.
![3](https://user-images.githubusercontent.com/29533107/148199485-8a29ba67-dbe4-4d3b-b968-d44183875582.jpg)

● Extract SIFT features from the undistorted planes.
![image](https://user-images.githubusercontent.com/29533107/148199979-826b4369-9862-4ed7-9f01-5843aef42d26.png)
![image](https://user-images.githubusercontent.com/29533107/148200011-31168dc0-0985-4f65-ae01-99d91c31757e.png)

● Match with features from the database and look for highest scores.

