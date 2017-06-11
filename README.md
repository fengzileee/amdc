# AMDC

Some notes here.

## Vision

Video feed from camera is obtained using openRTSP. The feed is saved to a file
which will then be read by the `vision_node` to perform object detection. 

Object detection is done by performing regular image classification with a CNN
over a sliding window. The neural network framework used is darknet.

#### openRTSP

https://gist.github.com/randolphwong/fc4077abc8c51ec5f0662d359869337e

#### darknet

https://pjreddie.com/darknet/

#### how to run

```
rosrun amdc vision_node data/cnn_configuration data/cnn_weights videofeed.mp4
```
