# pisa-iit-soft-hand (ROS/Gazebo packages)

This repository contains the model of the Pisa/IIT hand as described in:

* M. G. Catalano, Grioli, G., Farnioli, E., Serio, A., Piazza, C., and Bicchi, A., “Adaptive Synergies for the Design and Control of the Pisa/IIT SoftHand”, International Journal of Robotics Research, vol. 33, no. 5, pp. 768–782, 2014

[Free version of the paper](http://www.centropiaggio.unipi.it/sites/default/files/PisaIIT_SoftHand_0.pdf) and
[IJRR version (access required)](http://ijr.sagepub.com/content/33/5/768.abstract)

Unless stated otherwise, all files within the repository are released under the BSD 3-Clause License, see the [LICENSE](https://github.com/CentroEPiaggio/pisa-iit-soft-hand/blob/master/LICENSE) file for the details.

## Cloning the repository
```
git clone --recursive https://github.com/CentroEPiaggio/pisa-iit-soft-hand.git
```

## Dependencies

ToDo: Create a travis.yml file for this.

## Examples
There are several [examples](https://github.com/CentroEPiaggio/pisa-iit-soft-hand/tree/master/examples) that show how the hand can be used in different configurations.

###Push the finger (Not implemented)

Test the adaptive synergy transmission by applying a wrench to the middle fingertip (you must choose start_time and duration according to the ROS time clock)

```
rosservice call /gazebo/apply_body_wrench "body_name: 'soft_hand::soft_hand_middle_distal_link'
wrench:
  force: {x: 0.0, y: 0.0, z: 10.0}
  torque: {x: 0.0, y: 0.0, z: 0.0}
duration: {secs: -1, nsecs: 0}"
```

And clear the wrench in case you want to continue working normally

`rosservice call /gazebo/clear_body_wrenches "body_name: 'soft_hand::soft_hand_middle_distal_link'"`

## Hand configuration using QBtools (USB/Handle)

These packages assume you use qbTools to move the hand, since it is the electronics the hand is sold with.

To set-up the hand, refer to the instructions by [NMMI](https://github.com/NMMI/qbadmin).

The interface is now shared between the `SoftHand` and `qbMove` devices.
# pisa_hand
