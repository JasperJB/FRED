Key Features
✅ Dual Control Modes:

fred.py: Manual control mode for data collection via a PS4 controller.
fredAI.py: Autonomous mode using a TFLite model for inference-based driving.
✅ Real-time Sensor Data Logging:

Arduino Sensor Data (Heading & 4 Ultrasonic Distance Sensors).
Depth Camera Data (9x12 normalized distance matrix).
Motor Commands (for training AI models).
✅ AI-Based Driving:

Uses TensorFlow Lite (model_ai_camera_stacked.tflite).
Reads camera depth, heading, and distance sensors to predict motor actions.
✅ Debugging & Logging:

--debug flag for verbose logging.
Automatic logging of sensor data, motor states, and errors to Redis and CSV.
