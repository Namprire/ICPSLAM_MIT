# SLAM Project

This project implements a simple Simultaneous Localization and Mapping (SLAM) algorithm using the Iterative Closest Point (ICP) method for point cloud alignment. The project includes visualization capabilities using Matplotlib to animate the robot's path and the map being built.

## Project Structure

```
slam-project
├── src
│   ├── __init__.py
│   ├── icp.py
│   ├── slam.py
│   └── visualization.py
├── data
│   └── README.md
├── tests
│   ├── __init__.py
│   ├── test_icp.py
│   └── test_slam.py
├── requirements.txt
└── README.md
```

## Installation

To install the required dependencies, run:

```
pip install -r requirements.txt
```

## Usage

1. **Run the SLAM Algorithm**: You can run the SLAM algorithm by executing the `slam.py` script in the `src` directory. Make sure to provide the necessary input data.

2. **Visualize the Results**: Use the `visualization.py` module to create animations of the SLAM process. The `animate` function will help visualize the robot's path and the evolving map.

## Testing

Unit tests for the ICP and SLAM implementations are located in the `tests` directory. You can run the tests using:

```
pytest tests/
```

## Data

The `data` directory contains datasets required for testing the SLAM implementation. Please refer to the `data/README.md` file for more information on the data formats and usage.

## Contributing

Contributions are welcome! Please feel free to submit a pull request or open an issue for any suggestions or improvements.

## License

This project is licensed under the MIT License. See the LICENSE file for more details.