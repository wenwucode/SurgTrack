## Installation

This repository has been tested on Python 3 as well and a wide range of intel realsense drivers.
```
sudo apt install build-essential cmake git pkg-config libssl-dev libgl1-mesa-glx
sudo apt-get install meshlab
pip install numpy pypng scipy scikit-learn open3d scikit-image tqdm pykdtree opencvpython==4.6.0.66 opencv-contrib-python==4.6.0.66 trimesh pyrealsense2 matplotlib
pyyaml plyfile
```
## Data set production
Print out all the QR codes in the arucomarker folder, and stick them in a circle around the object to be photographed. Try to stick them all, and stick them flatly.
Execute the following command to start shooting. When shooting, the camera or positioning plate should rotate at a constant speed as much as possible.
```
cd tools
python record2.py LINEMOD/obj
```
Calculate the displacement and rotation of each frame relative to the first frame.
```
python compute_gt_poses.py LINEMOD/obj
```
Open the LINEMOD/obj_name folder to find the registeredScene.ply file, open it with meshlab, and remove the background point cloud. At the same time, complete the hole filling and flattening operations to obtain the mesh of the object.

Execute the following commands to complete the creation of the mask and visualize the effect.
```
python create_label_files.py LINEMOD/obj
python inspectMasks.py LINEMOD/obj
```
## Normalization of data sets
```
# Copy all the .py files in the scripts folder to the LINEMOD/obj folder, and run the script to generate the displacement and rotation matrices.
python gt_file.py
# Convert jpg images to png, four-digit naming format.
python png_rename.py
# Generate script
python info.py
# Script formatting
python re-format.py
# Generate training and testing txt files
python train_test_txt.py

```
