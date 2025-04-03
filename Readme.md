To run agent

```bash
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:jazzy udp4 --port 8888
```

send message to feed

```
ros2 topic pub /SmartFeederInput --once std_msgs/msg/Int32 "{data: -1}"
```

to change portion size

```
ros2 topic pub /SmartFeederInput --once std_msgs/msg/Int32 "{data: 200}"
```

where 200 - number of steps for stepper motor

# TODO

- write your network settings into ESP_code/SmartFeeder/src/main.cpp
- download files for yolov3: [yolov3.weights](https://pjreddie.com/media/files/yolov3.weights), [yolov3.cfg](https://github.com/pjreddie/darknet/blob/master/cfg/yolov3.cfg?raw=true), [coco.names](https://github.com/pjreddie/darknet/blob/master/data/coco.names?raw=true)
    to download you can use wget:
    ```bash
    wget https://pjreddie.com/media/files/yolov3.weights
    wget https://github.com/pjreddie/darknet/blob/master/cfg/yolov3.cfg?raw=true
    wget https://github.com/pjreddie/darknet/blob/master/data/coco.names?raw=true
    ```
- provide path to files for the VideReaderPublisher node