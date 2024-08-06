# Stereo Vision Using Disparity Map

## Overview

Stereo vision is a technique used to obtain depth information from two or more images of the same scene taken from slightly different viewpoints. By analyzing the disparity between these images, we can estimate the 3D structure of the scene. This repository focuses on implementing disparity mapping to achieve depth estimation using stereo vision.


![Example Disparity Map](/media/illustrate.png)


## The Idea

The core idea behind stereo vision is to simulate human depth perception by using two cameras placed at different angles. When the images from these cameras are compared, the differences (or disparities) between corresponding points in the images can be used to calculate the depth of objects in the scene. This process is essential for various applications, including:

- **Depth Estimation for Robotics**: Providing robots with the ability to understand the 3D structure of their environment for navigation and interaction.
- **3D Reconstruction for 3D Modeling**: Creating detailed 3D models of real-world objects or environments for use in simulations, design, and visualization.
- **Augmented Reality**: Enhancing the user experience by overlaying digital information onto the physical world based on accurate depth data.

## Why Use Disparity Mapping?

Disparity mapping allows for accurate 3D reconstruction of a scene by leveraging the differences between stereo image pairs. This method is widely used because it provides valuable depth information and can be implemented with relatively straightforward algorithms, making it suitable for various practical applications.

## Disparity Mapping Process

1. **Image Acquisition**: Simultaneous capture of left and right images using stereo cameras.
2. **Rectification**: Aligning the image pairs to simplify the correspondence problem to a 1D search along epipolar lines.
3. **Stereo Matching**: Identifying corresponding points between the left and right images. Common algorithms include:
   - **Block Matching (BM)**
   - **Semi-Global Block Matching (SGBM)**
   - **Graph Cut**
4. **Disparity Computation**: Calculating the disparity for each matched point pair to determine depth.
5. **Post-processing**: Refining the disparity map through techniques like interpolation, filtering, and consistency checking to improve accuracy.


![Example Disparity Map](/media/math1.png)

![Example Disparity Map](/media/geom.png)

## Results

![Example Disparity Map](/media/left_eye_30cm.jpg)
![Example Disparity Map](/media/right_eye_30cm.jpg)

![Example Disparity Map](path/to/your/image.png)
