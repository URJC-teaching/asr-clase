# Copyright 2023 Rodrigo Pérez-Rodríguez
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from llama_bringup.utils import create_llama_launch
import yaml

def generate_launch_description():
    
    whisper = True
    llama = True
    
    # Get the launch directories for other packages
    whisper_dir = get_package_share_directory('whisper_bringup')

    llama_cmd = create_llama_launch(
            n_ctx=2048,
            n_batch=256,
            n_gpu_layers=23,#0, 23
            n_threads=4,
            n_predict=-1,

            model_repo='TheBloke/Marcoroni-7B-v3-GGUF',
            model_filename='marcoroni-7b-v3.Q3_K_L.gguf',
            # model_repo='TheBloke/phi-2-GGUF',
            # model_filename='phi-2.Q4_K_M.gguf',

            prefix='\n\n### Instruction:\n',
            suffix='\n\n### Response:\n',
            stopping_words=["\n\n\n\n"],
    )

    whisper_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(whisper_dir, 'launch', 'whisper.launch.py')
        ),
        launch_arguments={'stream': 'False',
                          'language': 'es',
                          'model_repo': 'ggerganov/whisper.cpp',
                          'model_filename': 'ggml-large-v3-turbo-q5_0.bin',
                          'use_gpu': 'True',} # False
                          .items()                   
    )
    
    audio_common_player_cmd = Node(
        package='audio_common',
        executable='audio_player_node',
        parameters=[
            {'channels': 1},
            {'device': -1}]
    )
    # tts_model = 'tts_models/en/ljspeech/glow-tts'
    # tts_model = 'tts_models/es/mai/tacotron2-DDC'
    tts_model = 'tts_models/es/css10/vits'

    audio_common_tts_cmd = Node(
        package='tts_ros',
        executable='tts_node',
        parameters=[
            {'chunk': 4096},
            {'frame_id': ''},
            {'model': tts_model},
            {'speaker_wav': ''},
            {'device': 'cuda:0'}] # cpu
    )

    ld = LaunchDescription()
   
    if whisper:
        ld.add_action(whisper_cmd)
    if llama:
        ld.add_action(llama_cmd)
    ld.add_action(audio_common_player_cmd)
    ld.add_action(audio_common_tts_cmd)

    return ld

