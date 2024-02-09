### Create a workspace
```
mkdir -p ~/ros/galil/src
cd ~/ros/galil
```

### Clone repository
```git clone git@github.com:simonleonard/galil.git src/galil```

### Build
```colcon build```

### Run
```
source install/setup.bash
ros2 launch galil_driver view.launch.py
```

### Stream positions
```
ros2 topic echo /joint_states
```
### Position Command
```
cd src/galil/galil_driver/scripts
chmod 755 move_position.sh
./move_position.sh <encoder_cnt_x> <encoder_cnt_y> <encoder_cnt_z>
```
where the arguments are integers (specify all three)
