# sensors-and-control

Group 16:
- Luke Eyles
- Aaron Phan
- Parsa Jalili Tabaei

Project 6 - Driving turtlebot along normal of marker

Code outline:
- Initialise ROS 
    - Define ROS subscribers and publishers. 
    - Set global variables (odometry and twist messages) 
- Create timer to send twist messages periodically. 
- Retrieve initial odometry. 
- MAIN 
    - Define the original marker images to be referenced. 
    - Look for the markers in order. 
        - Check robot odometry for current pose. 
        - Set angle increment. 
            - Rotate to each increment. 
            - Read depth and RGB images and convert to MATLAB format. 
            - Look for and detect marker. 
                Load reference marker images 
                SURF Features algorithm 
                Transform and order corners. 
                Compute marker points. 
    - Compute global coordinates of marker 
        - Convert pixel coordinates to coordinates in camera frame. 
        - Convert coordinates in camera reference frame to global reference frame. 
    - Compute centre and normal of marker. 
    - Generate goal poses. 
        - Set offset from marker. 
        - Compute intersection point. 
    - Move robot to goals. 
        - Retrieve current odometry and extract location and orientation. 
        - Set error thresholds. 
        - Calculate angle and distance errors. 
        - Publish velocities. 
    - Go back to home position and repeat for remaining markers. 