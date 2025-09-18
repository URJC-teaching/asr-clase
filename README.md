# ASR

Este repositorio contiene código de ejemplo de Robótica Software.


1. Clona el repo en tu espacio de trabajo
```
mkdir -p asr_ws/src
cd asr_ws/src
git clone git@github.com:URJC-teaching/asr-clase.git
```

2. Instala las dependencias

```
cd asr_ws/src
vcs-import . < thirdparty.repos
cd ..
rosdep install --from-paths src --ignore-src -r
```

3. Construye

```
cd asr_ws/src
colcon build --symlink-install
```

## camera

1. Lanza YOLO
```
ros2 launch yolo_bringup yolo.launch.py input_image_topic:=/rgbd_camera/image input_depth_topic:=/rgbd_camera/depth_image input_depth_info_topic:=/rgbd_camera/camera_info target_frame:=camera_link
```

2. Lanza el nodo que transforma los mensajes de YOLO en mensajes estándar
```
ros2 launch camera yolo_detection.launch.py
```

## vff_control

1. Lanza YOLO
```
ros2 launch yolo_bringup yolo.launch.py input_image_topic:=/rgbd_camera/image input_depth_topic:=/rgbd_camera/depth_image input_depth_info_topic:=/rgbd_camera/camera_info target_frame:=camera_link
```

2. Lanza el nodo que transforma los mensajes de YOLO en mensajes estándar
```
ros2 launch camera yolo_detection.launch.py
```

3. Lanza la aplicación
```
ros2 launch vff_control vff_2d.launch.py 
```


## Licencia
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0)
