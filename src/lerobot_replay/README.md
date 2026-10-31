# LeRobot pose decoding

Pure numerical decoding, with no ROS or robot IO. Supports 27D/78D base_link
XYZ+rot6d storage at 20 Hz, and legacy chunk_first_frame_camera episodes with
verified anchors. Base-frame data never needs chunk_frames.json.

Named body/head controller coordinates retain their source units and joint order.
The client selects recorded final IK action streams, or applies a verified legacy
mapping contract. This decoder alone does not declare upstream teleop targets
ready for execution. Masks, timestamps and provenance are checked independently.
