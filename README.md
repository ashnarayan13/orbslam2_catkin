# orbslam2_catkin

Contains repository to build orb_slam2 with catkin_make

# Dependencies

* Pangolin (https://github.com/stevenlovegrove/Pangolin)
* OpenCV

# How to

* `mkdir -p ~/catkin_ws/src`
* `cd catkin_ws/src`
* `git clone https://github.com/ashnarayan13/orbslam2_catkin.git`
* `cd ..`
* `catkin_make`
* `source ./devel/setup.bash`

To run orbslam2 
* The ORBvoc.txt has to be extracted from the tar file. `tar -xzf ORBvoc.txt.tar.gz` to unzip the vocabulary.

* `rosrun orb_slam orb_slam /Vocabulary/ORBvoc.txt /path/to/camera_parameters.yaml true`

Related Publications:

[1] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. ORB-SLAM: A Versatile and Accurate Monocular SLAM System. Submitted to IEEE Transactions on Robotics. arXiv preprint: http://arxiv.org/abs/1502.00956

ORB-SLAM is released under a GPLv3 license. Please note that we provide along ORB-SLAM a modified version of g2o and DBoW2 which are both BSD. 

For a closed-source version of ORB-SLAM for commercial purposes, please contact the authors. 

If you use ORB-SLAM in an academic work, please cite:

@article{murSubTro2015,
  title={{ORB-SLAM}: a Versatile and Accurate Monocular {SLAM} System},
  author={Mur-Artal, Ra\'ul, Montiel, J. M. M. and Tard\'os, Juan D.},
  journal={Submitted to IEEE Transaction on Robotics. arXiv preprint arXiv:1502.00956},
  year={2015}
}
