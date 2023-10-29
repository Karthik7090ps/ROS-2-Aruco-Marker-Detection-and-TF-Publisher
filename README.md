# ROS 2 Aruco Marker Detection and TF Publisher

Welcome to the ROS 2 Aruco Marker Detection and TF Publisher repository! This Python code enables the detection of Aruco markers in ROS 2, computes their 3D positions, and publishes the resulting transformations as TF data. Aruco markers are commonly used in robotics and computer vision for pose estimation and tracking.

## Features

- Aruco marker detection and tracking using OpenCV.
- Calculation of 3D positions for detected markers.
- TF (Transform) broadcasting for ROS 2, facilitating integration with robotic systems.
- Visualization of detected markers on the input image.

## Prerequisites

- ROS 2: Make sure you have ROS 2 installed on your system.

## Usage

1. Clone or fork this repository.
2. Customize the camera calibration parameters in `aruco_tf_publisher.py` to match your hardware setup if necessary.
3. Run the Python script, `aruco_tf_publisher.py`, to start detecting Aruco markers and publishing TF data.

## Repository Contents

- `aruco_tf_publisher.py`: The main Python script for Aruco marker detection and TF publishing.
- `README.md`: This documentation file providing an overview of the repository and instructions for use.

## Contributing

Contributions to this repository are welcome! Feel free to submit pull requests, report issues, or suggest improvements. Let's work together to enhance the functionality and usability of this code for the ROS 2 community.

## License

This code is available under the [MIT License](LICENSE.md). You are free to use, modify, and distribute the code, but please review the full license for more details.

Feel free to explore, experiment, and integrate this code into your ROS 2 projects. If you have any questions or run into issues, please don't hesitate to contact us.

Happy coding and robotics with ROS 2! 🤖🌟
