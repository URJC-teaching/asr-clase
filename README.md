# asr-py

Este repositorio contiene código de ejemplo de Robótica Software.


1. Clona el repo en tu espacio de trabajo
```
mkdir asr_ws/
git clone git@github.com:URJC-teaching/asr-clase.git -b py
mv asr-clase/ src/
```

2. Instala las dependencias necesarias

```
cd asr_ws/src
vcs-import . < thirdparty.repos
cd ..
rosdep install --from-paths src --ignore-src -r
```
3. Crea un entorno virtual para Python y actívalo

```
python3 -m venv --system-site-packages venv_asr
echo -e '\n# Add venv_asr site-packages to PYTHONPATH\nVENV_SITE_PACKAGES="$VIRTUAL_ENV/lib/python3.12/site-packages"\nif [ -z "$PYTHONPATH" ]; then\n    export PYTHONPATH="$VENV_SITE_PACKAGES"\nelse\n    export PYTHONPATH="$VENV_SITE_PACKAGES:$PYTHONPATH"\nfi' >> venv_asr/bin/activate 
cd venv_asr/
touch COLCON_IGNORE
cd ..
source venv_asr/bin/activate
```


4. Instala las dependencias de *thirdparties*

```
pip3 install -r src/thirdparty/llama_ros/requirements.txt
pip3 install -r src/thirdparty/yolo_ros/requirements.txt
pip3 install -r src/thirdparty/tts_ros/requirements.txt
pip3 install -r src/thirdparty/simple_hri/requirements.txt
```

5. Construye

```
colcon build --symlink-install
```

## camera

1. Lanza YOLO
```
ros2 launch yolo_bringup yolo.launch.py input_image_topic:=/rgbd_camera/image input_depth_topic:=/rgbd_camera/depth_image input_depth_info_topic:=/rgbd_camera/camera_info target_frame:=camera_link
```

Si se está utilizando la cámara **oak**, primero hay que lanzarla:

```
ros2 launch oak_d_camera camera.launch.py use_disparity:=False use_lr_raw:=False use_pointcloud:=False
```

Y luego lanzar YOLO con los *remappings* correspondientes:

```
ros2 launch yolo_bringup yolo.launch.py input_image_topic:=/color/image input_depth_topic:=/stereo/depth input_depth_info_topic:=/stereo/camera_info target_frame:=oak-d_frame
```

2. Lanza el nodo que transforma los mensajes de YOLO en mensajes estándar
```
ros2 launch camera yolo_detection2d.launch.py # yolo_detection3d.launch.py
```

## vff_control

1. Lanza YOLO
```
ros2 launch yolo_bringup yolo.launch.py input_image_topic:=/rgbd_camera/image input_depth_topic:=/rgbd_camera/depth_image input_depth_info_topic:=/rgbd_camera/camera_info target_frame:=camera_link # use_3d:=True
```

Si se está utilizando la cámara **oak**, primero hay que lanzarla:

```
ros2 launch oak_d_camera camera.launch.py use_disparity:=False use_lr_raw:=False use_pointcloud:=False
ros2 run tf2_ros static_transform_publisher --child-frame-id oak-d-base-frame --frame-id base_footprint --x 0.0 --y 0.0 --z 1.0
```
Y luego lanzar YOLO con los *remappings* correspondientes:

```
ros2 launch yolo_bringup yolo.launch.py input_image_topic:=/color/image input_depth_topic:=/stereo/depth input_depth_info_topic:=/stereo/camera_info target_frame:=oak-d_frame # use_3d:=True
```

2. Lanza el nodo que transforma los mensajes de YOLO en mensajes estándar
```
ros2 launch camera yolo_detection2d.launch.py # yolo_detection3d.launch.py
```

3. Lanza la aplicación
```
ros2 launch vff_control vff_2d.launch.py # vff_3d.launch.py 
```
## hri_examples

### Modelos en la nube (Google & OpenAI)

1. Lanza los servidores STT y TTS así como otras dependencias necesarias
```
ros2 launch simple_hri simple_hri.launch.py
```

2. Lanza el ejemplo:
```
ros2 run hri_examples hri_example
```

### Modelos en local

1. Lanza todas las dependencias necesarias
```
ros2 launch hri_examples hri_dependencies.launch.py
```

2. Lanza el ejemplo que quieras probar:

```
ros2 run hri_examples say
```
```
ros2 run hri_examples repeat
```
```
ros2 launch hri_examples generate_response
```

## bt_bumpgo

Si queremos utilizar BTs editados con Groot, hay que ejecutar el siguiente comando para instalar `py_trees_meet_groot`

```
python3 -m pip install --user git+https://github.com/narcispr/py_trees_meet_groot.git
```

## Licencia
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0)
