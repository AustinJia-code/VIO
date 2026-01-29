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
Visualizer: ```python ../tools/Plotter.py```

### Issues:
* IMU physics fried
* EKF fried
* Cam seems to break down when initial features are lost?
    * Verified from time 1403636661163555584, starting from beginning it stops
      tracking here, but starting exactly on, tracking works.

### Future Work:
* Improve EKF
* Thread
* Tests