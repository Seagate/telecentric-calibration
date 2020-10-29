# Telecentric Calibration
Function routines for calibrating cameras with telecentric (orthographic) lenses using a group of checkerboard images. 
## Requirements
MATLAB's image processing, computer vision and optimization toolboxes.
## Usage
Two examples are provided: *SimpleOrthographicCalibration.m* contains a simple script for calibrating with a group of images. *batch_calibration_v1.m* contains sample usage for more advanced function routines.
## Notes
- Extrinsic orientations are chosen arbitrarily when there's ambiguity
- Avoid degeneracy during image acquistion (i.e. rotate the target around all 3 axes)
## Authors
Kaijun Feng
