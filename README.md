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

## Licencia
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
