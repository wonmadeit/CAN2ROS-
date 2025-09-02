# CAN to ROS Topic Converter

This ROS1 package allows you to receive data from the **SocketCAN** bus, decode it based on a **DBC** file, and publish it as **ROS topics**. It extracts specific signals and converts them into a format that can be used in ROS.

## Installation

To use this package, you need to install the required dependencies first.

### 1. Install Dependencies

Install the required libraries (`setuptools`, `python-can`, and `cantools`) using the following command:

```bash
pip install setuptools python-can cantools

