# README for Data in SLAM Project

This directory contains data files used for testing and validating the SLAM implementation. 

## Data Description

- **Point Cloud Data**: The project utilizes point cloud datasets for testing the Iterative Closest Point (ICP) algorithm. These datasets are typically in `.txt` or `.csv` format, containing 3D coordinates.

- **Map Data**: The SLAM algorithm requires map data to update the robot's position and build the environment map. This data can be generated from the point cloud data or can be provided as pre-existing datasets.

## Data Format

- Each point cloud file should contain rows of 3D coordinates (x, y, z), separated by commas or spaces.
- Ensure that the data is clean and free of noise for optimal performance of the SLAM algorithm.

## Usage

To use the data in your SLAM implementation, load the datasets into your Python scripts using appropriate libraries such as NumPy or Pandas. 

Make sure to follow the project structure and reference the data files correctly in your code.