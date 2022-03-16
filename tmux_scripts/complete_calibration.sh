#!/bin/bash

rostopic pub -1 "/$UAV_NAME/basler_stereo_driver/uav1/complete std_msgs/"Bool True
