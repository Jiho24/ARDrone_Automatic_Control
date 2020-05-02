# Introduction

### Goal: Developing Target Tracking And Avoidance System Based On Image Processing

  This project is executed in 2014, by ACUS LAB(AeroSpace Control & Unmanned System) in Hanseo University.
Used Open Sources for the project. Further Version is not expected to be released. Uploading for recording purpose.

# Content

#### - Image Processing Flow Chart
#### - PD Control
#### - Function to update current status of an Airframe(Velocity, Angle of yaw and roll)
#### - GCS(Ground Control System) with Matlab GUI
#### -	Target Tracking 
#### -	Target Avoidance



# Flow Chart
![Screen Shot 2020-05-02 at 12 26 12 am](https://user-images.githubusercontent.com/44355683/80846131-9fb21b00-8c0b-11ea-9e4b-a7c409fd7113.png)

We got images from a camera on ARDrone and processed it on real-time by using UDP communication method. Through the processing steps, target object is detected and object-center value is calculated. Values are passed to Control Team.

# PD Control
![Screen Shot 2020-05-02 at 1 39 29 am](https://user-images.githubusercontent.com/44355683/80849043-ce34f380-8c15-11ea-9d85-3b34e3d1ae65.png)

We used PD Control to control the ARDrone. Control team calculated the error distance from the object-center to target value, then used PD(Proportion&Differentiation) Control method to approach the target value.
# Airframe Status

![Screen Shot 2020-05-01 at 11 58 49 pm](https://user-images.githubusercontent.com/44355683/80845898-eeab8080-8c0a-11ea-8964-4f3dca1605d1.png)

Developed function for updating a airframe's status. 
It shows current position of the airplane, velocity, theta(angle) and battery... etc)

# GCS(Ground Control System) with Matlab GUI
![Screen Shot 2020-04-29 at 2 01 36 am](https://user-images.githubusercontent.com/44355683/80847331-827f4b80-8c0f-11ea-9b64-0e2af45cb56b.png)

GUI(Graphic User Interface) for easy execution of the program. It contains functions for ground control such as UDP Control, Key Control, Emergency and Motor Check.

# Target Tracking and Target Avoidance
As a result of the project, we successfully developed these functions. 
Experiment videos are uploaded on this repository.  
