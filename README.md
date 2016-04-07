# perception_neuron_gesture_recognition
ROS node providing hand and head gesture recognition for the Perception Neuron.

To get the data from the Perception Neuron the ROS package from Simon Haller is necessary, which can be found here: [https://github.com/smhaller/perception-neuron-ros](https://github.com/smhaller/perception-neuron-ros). 

## Node
The node detects different hand and head gestures, which are published to the topics:
  - **/perception/hand_gestures**   detected hand gestures
  - **/perception/head_gestures**   detected head gestures
  - **/perception/raw/head_orientation**  raw head orientation data
  
For initialization it is necessary to hold the initialization gesture for 5 seconds and call the service **/perception/init**. This gesture is sitting on a chair, looking straight ahead and putting the hands in front of one on a table. According, to how good this initialization was done, the gestures are detected well or not.
