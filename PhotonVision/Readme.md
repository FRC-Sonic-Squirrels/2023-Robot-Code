# PhotonVision

## Howto Documentation

OrangePi 5 setup instructions:
https://docs.google.com/document/d/1lOSzG8iNE43cK-PgJDDzbwtf6ASyf4vbW8lQuFswxzw/edit

Belink PC setup instructions, includes useful camera information that is useful for any PhotonVision setup:
https://docs.google.com/document/d/17DNCNHxUo31Rh-7VmXXyn-Y25UtGND3NPoGL9gRosaQ/edit

Here's a link that describes how to change the serial number and device name for Arducam cameras. If you want to run more than one of the same camera type on an OrangePi, you have to change the serial number and/or device name so that PhotonVision can tell them apart.
https://docs.arducam.com/UVC-Camera/Serial-Number-Tool-Guide/

## Cameras we Use

https://www.arducam.com/product/arducam-120fps-global-shutter-usb-camera-board-1mp-720p-ov9281-uvc-webcam-module-with-low-distortion-m12-lens-without-microphones-for-computer-laptop-android-device-and-raspberry-pi/

https://www.arducam.com/product/arducam-2mp-global-shutter-usb-camera-board-for-computer-50fps-ov2311-monochrome-uvc-webcam-module-with-low-distortion-m12-lens-without-microphones-compatible-with-windows-linux-android-and-mac-os/

## Camera Calibration

To use 3d AprilTag tracking you will need to calibrate the camera lens. One option is to get the camera calibration data from the calibdb.net. The calibration files for our cameras are in the `calibdb` directory in this repo.
