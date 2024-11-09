cd ../..
source install/setup.bash
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/Qt/6.8.0/gcc_64/lib ros2 run drone_control drone_controller
