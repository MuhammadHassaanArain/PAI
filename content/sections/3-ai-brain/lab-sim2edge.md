# Lab 3.3: Sim-to-Edge Inference: Deploying AI to Jetson

**Chapter ID**: 3.3
**Module**: AI Brain – NVIDIA Isaac
**Authors**: Gemini Agent
**Last Updated**: December 4, 2025

## 🎯 Learning Objectives

By the end of this lab, you will be able to:
*   Understand the concept of Sim-to-Edge deployment in robotics.
*   Prepare a trained AI model for deployment on an NVIDIA Jetson device.
*   Deploy an inference pipeline developed in simulation to an edge device.
*   Verify the model's performance and functionality on the Jetson.

## ✨ Core Concepts

### Sim-to-Edge Deployment

Sim-to-Edge deployment refers to the process of developing and training AI models within a high-fidelity simulation environment (like Isaac Sim) and then deploying these models to real-world edge computing devices (like NVIDIA Jetson platforms) for inference. This workflow is crucial in robotics because it allows for rapid iteration, safe testing, and the generation of vast amounts of synthetic data for training, all without the risks and costs associated with real hardware. Once validated in simulation, the trained models can be optimized and pushed to the edge for real-time operation.

#### Key Terms
*   **Sim-to-Edge**: A development workflow where AI models are trained in simulation and deployed to edge devices.
*   **Edge Device**: A computing device located close to the source of data (e.g., a robot's onboard computer).
*   **Inference Pipeline**: A series of steps to process sensor data and run an AI model to make predictions.

### Preparing Models for Edge Deployment

Models trained in simulation often need optimization for efficient deployment on edge devices, which typically have constrained computational resources compared to training servers. This often involves:
*   **Model Quantization**: Reducing the precision of model weights (e.g., from floating-point to INT8) to reduce model size and speed up inference.
*   **TensorRT Optimization**: NVIDIA's TensorRT is an SDK for high-performance deep learning inference. It optimizes trained neural networks for various NVIDIA GPUs, including Jetson, by fusing layers, optimizing kernel selection, and managing memory.
*   **Deployment Formats**: Converting models to formats suitable for edge deployment, such as ONNX or directly to TensorRT engines.

### The Inference Pipeline on Jetson

On a Jetson device, an inference pipeline typically involves:
1.  **Sensor Input**: Capturing real-time data from onboard sensors (e.g., camera, LiDAR).
2.  **Pre-processing**: Preparing the sensor data for the AI model (e.g., resizing images, normalization).
3.  **Model Inference**: Running the optimized AI model (e.g., a perception model for object detection) on the Jetson's GPU.
4.  **Post-processing**: Interpreting the model's output (e.g., drawing bounding boxes, converting predictions into ROS 2 messages).
5.  **Output**: Sending processed information to other robot systems (e.g., a navigation stack, a manipulation controller) via ROS 2.

## 💻 Code Examples

This lab focuses on the deployment aspect. The AI model itself would have been trained in a prior simulation-based task. Here, we'll demonstrate a conceptual Python script for running a simple object detection model on Jetson.

### Example: Conceptual Object Detection Inference on Jetson

This pseudo-code demonstrates a simplified inference script that might run on a Jetson device, utilizing an optimized model.

**Assumptions**: An object detection model (e.g., YOLOv5, DetectNet) has been trained in simulation, optimized with TensorRT, and deployed to the Jetson.

```python
# Conceptual Python Code for Jetson Inference
import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

# Initialize TensorRT engine (pre-built from optimized model)
TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
def load_trt_engine(trt_engine_path):
    with open(trt_engine_path, 'rb') as f, trt.Runtime(TRT_LOGGER) as runtime:
        return runtime.deserialize_cuda_engine(f.read())

engine = load_trt_engine('optimized_object_detector.trt')
context = engine.create_execution_context()

# Allocate buffers for input and output
# ... (simplified for conceptual example)

def preprocess_image(image_np):
    # Example: resize, normalize, convert to Cuda array
    # ...
    return processed_image_cuda

def postprocess_output(output_cuda):
    # Example: parse bounding boxes, confidence scores
    # ...
    return detections

def run_inference(image_frame):
    processed_input = preprocess_image(image_frame)
    # Perform inference (simplified)
    # context.execute_v2(bindings=[processed_input, output_buffer])
    output_buffer = np.random.rand(10, 5) # Placeholder for actual output
    detections = postprocess_output(output_buffer)
    return detections

# Main loop for camera input
def main():
    cap = cv2.VideoCapture(0) # Assume /dev/video0 is the camera
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    print("Running inference pipeline on Jetson...")
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Run inference
        detections = run_inference(frame)

        # Draw detections on frame (example)
        for det in detections:
            # ... draw bounding boxes ...
            pass
        cv2.imshow('Jetson Inference', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
```

*   **Expected Output**: This script, when deployed and run on a Jetson with a connected camera and a TensorRT-optimized model, would display a video feed with real-time object detections. The specific output would depend on the model and environment.

## 🧪 Step-by-Step Lab: Sim-to-Edge Object Detector Deployment

**Goal**: Deploy a pre-trained (in simulation) and TensorRT-optimized object detection model to an NVIDIA Jetson device and run real-time inference using a live camera feed.
**Duration**: 45 minutes
**Dependencies**: NVIDIA Jetson Orin Nano/NX/AGX, Ubuntu 20.04/22.04 with JetPack installed, a USB camera, a pre-trained and TensorRT-optimized object detection model (e.g., from NVIDIA's `isaac_ros_benchmark` examples or a custom model).

### 🛠️ Hardware / Cloud Alternative

*   **On-Premise**: NVIDIA Jetson Orin Nano (recommended), Jetson Orin NX, or Jetson AGX Orin.
*   **Cloud**: This lab specifically targets edge hardware. A cloud alternative would involve setting up a virtual Jetson environment, which is beyond the scope of this particular lab. However, understanding the deployment principles can be done conceptually.

### Instructions

#### Step 1: Prepare Your Jetson Environment

1.  **Ensure JetPack is installed**: Follow NVIDIA's documentation to ensure JetPack is installed on your Jetson device. This includes CUDA, cuDNN, and TensorRT.
2.  **Install necessary libraries**:
    *   `sudo apt update && sudo apt install -y python3-pip libopencv-dev`
    *   `pip3 install numpy pycuda`
    *   Install ROS 2 Humble if not already present.
    *   Install Isaac ROS components that include TensorRT inference examples (refer to NVIDIA Isaac ROS documentation).

#### Step 2: Transfer the Optimized Model

1.  Transfer your `optimized_object_detector.trt` file (or similar TensorRT engine file) to your Jetson device. A common location is `~/models/`.
    *   `scp /path/to/your/optimized_object_detector.trt user@jetson_ip:~/models/`

#### Step 3: Create the Inference ROS 2 Package

1.  On your Jetson, create a ROS 2 workspace and a new Python package (similar to Lab 1.2):
    *   `mkdir -p ~/jetson_ws/src`
    *   `cd ~/jetson_ws/src`
    *   `ros2 pkg create --build-type ament_python jetson_inference_pkg --dependencies rclpy cv_bridge sensor_msgs`
    *   `cd jetson_inference_pkg/jetson_inference_pkg`
2.  Create `jetson_detector.py` and adapt the conceptual code example above, ensuring it reads images from a ROS 2 camera topic (`/image_raw`) and publishes detected objects as ROS 2 messages (e.g., `detection_msgs/Detection2DArray`). This will require using `cv_bridge` to convert ROS images to OpenCV images.

#### Step 4: Implement the Inference Node (`jetson_detector.py`)

(Full Python code for ROS 2 node using `cv_bridge` and `TensorRT` will be provided here, building upon the conceptual example and assuming a specific Isaac ROS TensorRT example for integration).

#### Step 5: Update `setup.py`

Update `~/jetson_ws/src/jetson_inference_pkg/setup.py` to include `jetson_detector.py` in its `entry_points`.

#### Step 6: Build and Source

1.  `cd ~/jetson_ws`
2.  `colcon build`
3.  `source install/setup.bash`

#### Step 7: Run the Inference Pipeline

1.  Connect a USB camera to your Jetson.
2.  Launch a ROS 2 camera driver for your USB camera (e.g., `ros2 launch usb_cam usb_cam_node.launch.py`).
3.  Launch your inference node:
    *   `ros2 run jetson_inference_pkg jetson_detector`
4.  Verify detections:
    *   Use `rqt_image_view` to view the processed image topic or `ros2 topic echo /detections` to see detection messages.

## ⚠️ Safety Notes

*   Ensure proper power supply for your Jetson device and any connected peripherals. Insufficient power can lead to system instability.
*   Always test models thoroughly in simulation before deploying to a physical robot to minimize unexpected behaviors.
*   Monitor your Jetson's CPU/GPU temperatures and resource usage during inference to prevent overheating and throttling.

## 📚 Summary

*   Sim-to-Edge is a critical workflow for efficient AI robotics development.
*   Model optimization techniques like quantization and TensorRT are essential for edge deployment.
*   Jetson devices provide powerful inference capabilities at the edge.
*   Deploying models involves preparing the environment, transferring optimized models, and integrating inference into ROS 2 pipelines.

## 📝 Assessment / Mini Project

**Challenge**: Research and discuss how **transfer learning** can be effectively used in a Sim-to-Edge workflow to adapt models trained on synthetic data to perform better on real-world sensor data.

## 🤖 Agent Interaction Examples

The embedded RAG chatbot can assist with your learning. Here are some examples of how to interact with it for this chapter:

*   **Query**: "What is model quantization and why is it important for edge devices?"
*   **Query**: "How does TensorRT optimize neural networks for NVIDIA GPUs?"
*   **Query**: "What are the advantages of edge computing for robotics compared to cloud computing?"
*   **Query**: "Explain the typical steps in an AI inference pipeline on a Jetson."
