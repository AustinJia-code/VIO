### Dataset Pipeline
```
pip install opencv-python
pip install rosbags
bash install.sh     // 12GB
```

### Build:
```
cd build
cmake ..
make
```

### Run:
Live estimation: ```./live_play```\
Dataset playback: ```./data_play```

### Todo:
* Improve EKF
* Thread
* Tests