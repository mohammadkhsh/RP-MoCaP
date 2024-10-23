# MoCap_Project![0](https://media.git.i.mercedes-benz.com/user/29336/files/9bc85833-3135-46c2-80e4-aca53ee0c070)

# Automatic Annotation of Human 3D Poses in 2D Images Using MoCap

This project provides a set of tools for automating the annotation of human 3D poses in 2D images using Motion Capture (MoCap) data. The pipeline includes tools for recording, calibration checks, converting images to ROS bags, and offline image projection.

## Project Structure

The repository contains the following Python scripts:

### 1. `Automized_recorder.py`
This script automates the recording process for 3D pose data using a camera and MoCap system. It captures 2D images and associates them with 3D pose annotations, which can then be used for training machine learning models.

#### Key Features:
- Records 3D poses from MoCap system.
- Captures synchronized 2D images for each 3D pose.
- Automatically handles the storage and management of recorded data.

### 2. `calibration_check.py`
This script ensures that the camera is properly calibrated with the MoCap system. It performs a series of checks to verify the calibration's accuracy, crucial for maintaining the integrity of 3D pose annotations.

#### Key Features:
- Verifies calibration parameters.
- Provides visual feedback on calibration quality.
- Outputs calibration correction if necessary.

### 3. `jpg2rosbag.py`
This script converts a series of JPEG images into a ROS bag format, allowing for easy playback and analysis within the ROS framework. It's especially useful when working with ROS for robotic applications or post-processing data.

#### Key Features:
- Converts images to ROS bag format.
- Organizes image sequences into time-stamped data streams.

### 4. `projector_jpg_offline.py`
This script handles the offline projection of 3D MoCap data onto 2D images. It takes recorded 3D poses and projects them onto corresponding 2D images to create annotated datasets.

#### Key Features:
- Projects 3D poses onto 2D image planes.
- Supports batch processing for large datasets.
- Visualizes projected poses on the images.

## Setup

1. Clone the repository:
   ```bash
   git clone https://github.com/mohammadkhsh/RP-MoCap.git
   ```

2. Install dependencies:
   
## Usage

### 1. Running the Recorder
To record 2D images and their associated 3D MoCap poses, run:
\`\`\`bash
python Automized_recorder.py --output_dir /path/to/output
\`\`\`

### 2. Calibration Check
Verify the camera calibration using:
\`\`\`bash
python calibration_check.py --calibration_file /path/to/calibration
\`\`\`

### 3. Convert JPEG to ROS Bag
To convert images into a ROS bag format:
\`\`\`bash
python jpg2rosbag.py --input_dir /path/to/images --output /path/to/rosbag.bag
\`\`\`

### 4. Project 3D Poses onto 2D Images
For offline annotation projection:
\`\`\`bash
python projector_jpg_offline.py --pose_file /path/to/poses --image_dir /path/to/images --output /path/to/annotated_images
\`\`\`

## License

This project is licensed under the MIT License.
