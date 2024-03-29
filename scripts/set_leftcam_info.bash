#!/bin/bash

# husky_Mar_21
# +++++++++++++++++++
# rosservice call /stereo/left/set_camera_info "camera_info:
#   header:
#     seq: 0
#     stamp: {secs: 0, nsecs: 0}
#     frame_id: ''
#   height: 0
#   width: 0
#   distortion_model: 'plumb_bob'
#   D: [-0.019285233997575087, 0.035947833861205765, 6.0127458205990066e-05, -0.0016491603249031042, 0.0]
#   K: [725.2136520469213, 0.0, 627.0286674430602, 0.0, 723.066054945989, 527.8653973966301, 0.0, 0.0, 1.0]
#   R: [0.9994868987428702, 0.0038457517344127746, 0.031798575989446666, -0.003781914231311271, 0.9999907113236656, -0.002067460068860445, -0.03180623156091205, 0.0019461397654301148, 0.9994921591357818]
#   P: [771.9639227128613, 0.0, 581.486457824707, 0.0, 0.0, 771.9639227128613, 521.8288040161133, 0.0, 0.0, 0.0, 1.0, 0.0]
#   binning_x: 0                                                             
#   binning_y: 0                                                             
#   roi: {x_offset: 0, y_offset: 0, height: 0, width: 0, do_rectify: false}" 

# husky_Sep15_22_0
# +++++++++++++++++++
rosservice call /stereo/left/set_camera_info "camera_info:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  height: 1024
  width: 1224
  distortion_model: 'plumb_bob'
  D: [-0.071430, 0.036078, -0.004514, -0.000722, 0.000000]
  K: [670.95818,   0.     , 617.95349,
           0.     , 673.34226, 518.92384,
           0.     ,   0.     ,   1.     ]
  R: [ 0.99956084, -0.00588364,  0.02904321,
          0.00569023,  0.99996111,  0.00673752,
         -0.02908172, -0.0065693 ,  0.99955545]
  P: [681.99707,   0.     , 596.75923,   0.     ,
           0.     , 681.99707, 503.14468,   0.     ,
           0.     ,   0.     ,   1.     ,   0.     ]
  binning_x: 0                                                             
  binning_y: 0                                                             
  roi: {x_offset: 0, y_offset: 0, height: 0, width: 0, do_rectify: true}" 
