#!/bin/bash

rostopic pub -1 "/$UAV_NAME/basler_stereo_driver/$UAV_NAME/complete std_msgs/"Bool True
