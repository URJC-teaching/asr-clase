import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory

import os

import json

from llama_msgs.action import GenerateResponse
from audio_common_msgs.action import TTS
from whisper_msgs.action import STT


class GenerateResponseNode(Node):

    def __init__(self):
        super().__init__('generate_response_node')

        self.llama_client = ActionClient(self, GenerateResponse, '/llama/generate_response')
        self.tts_client = ActionClient(self, TTS, 'say')
        self.stt_client = ActionClient(self, STT, 'whisper/listen')

        self.declare_parameter('prompt_file', 'prompt.txt')
        self.declare_parameter('grammar_file', 'grammar.txt')
        self.declare_parameter('placeholder', '[]')
        self.declare_parameter('initial_prompt', '¿Qué quieres beber?')
        self.declare_parameter('intention', 'order_drink')

        self.prompt_file = self.get_parameter('prompt_file').get_parameter_value().string_value
        self.grammar_file = self.get_parameter('grammar_file').get_parameter_value().string_value
        self.placeholder = self.get_parameter('placeholder').get_parameter_value().string_value
        self.initial_prompt = self.get_parameter('initial_prompt').get_parameter_value().string_value
        self.intention_input = self.get_parameter('intention').get_parameter_value().string_value

        self.get_logger().info(f'Using prompt file: {self.prompt_file}')
        self.get_logger().info(f'Using grammar file: {self.grammar_file}')
        self.get_logger().info(f'Using placeholder: "{self.placeholder}"')
        self.get_logger().info(f'Initial prompt: "{self.initial_prompt}"')
        self.get_logger().info(f'Intention input: "{self.intention_input}"')

        self.state = 'INIT'        

        self.start()

    def start(self):
        if not self.llama_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Llama server not available.')
            return
        if not self.tts_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('TTS server not available.')
            return
        if not self.stt_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('STT server not available.')
            return
        self.transition_to('SAY_PROMPT')

    def transition_to(self, new_state):
        self.get_logger().info(f'Transitioning to state: {new_state}')
        self.state = new_state

        if self.state == 'SAY_PROMPT':
            self.say_prompt()
        elif self.state == 'LISTEN':
            self.listen()
        elif self.state == 'GENERATE':
            self.prepare_and_send_prompt()
        elif self.state == 'PARSE':
            self.parse_response()
        elif self.state == 'SAY_RESULT':
            self.say_intention()
        elif self.state == 'DONE':
            rclpy.shutdown()

    def say_prompt(self):
        goal = TTS.Goal()
        goal.text = self.initial_prompt
        self.get_logger().info(f'Speaking: "{goal.text}"')
        self.tts_client.send_goal_async(goal).add_done_callback(self.tts_prompt_callback)

    def tts_prompt_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Prompt goal was rejected.')
            self.transition_to('DONE')
            return
        goal_handle.get_result_async().add_done_callback(lambda _: self.transition_to('LISTEN'))

    def listen(self):
        goal = STT.Goal()
        self.get_logger().info('Listening...')
        self.stt_client.send_goal_async(goal).add_done_callback(self.stt_goal_callback)

    def stt_goal_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('STT goal was rejected.')
            self.transition_to('DONE')
            return
        goal_handle.get_result_async().add_done_callback(self.stt_result_callback)

    def stt_result_callback(self, future):
        result = future.result().result
        self.transcribed_text = result.transcription.text if hasattr(result.transcription, 'text') else ''
        self.get_logger().info(f'Heard: "{self.transcribed_text}"')

        if not self.transcribed_text.strip():
            self.get_logger().warn('No valid transcription received.')
            self.transition_to('DONE')
            return

        self.transition_to('GENERATE')

    def prepare_and_send_prompt(self):
        self.goal = GenerateResponse.Goal()
        self.result = None
        self.intention = ''
        

        try:
            package_share = get_package_share_directory('hri_examples')
            self.prompt_path = os.path.join(package_share, 'config', self.prompt_file)
            self.grammar_path = os.path.join(package_share, 'config', self.grammar_file)
        except Exception as e:
            self.get_logger().error(f'Could not resolve package path: {e}')
            self.transition_to('DONE')
            return

        prompt = self.load_text_file(self.prompt_path)
        grammar = self.load_text_file(self.grammar_path)

        if not prompt or not grammar:
            self.get_logger().error('Error loading prompt or grammar.')
            self.transition_to('DONE')
            return

        prompt = self.swap_placeholders(prompt, [self.transcribed_text, self.intention_input])

        self.goal = GenerateResponse.Goal()
        self.goal.prompt = prompt
        self.goal.reset = True
        self.goal.sampling_config.temp = 0.0
        self.goal.sampling_config.grammar = grammar

        self.get_logger().info(f'Sending prompt:\n{prompt}')
        self.llama_client.send_goal_async(self.goal).add_done_callback(self.llama_goal_callback)

    def llama_goal_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Llama goal was rejected.')
            self.transition_to('DONE')
            return
        goal_handle.get_result_async().add_done_callback(self.llama_result_callback)

    def llama_result_callback(self, future):
        self.result = future.result().result
        self.transition_to('PARSE')

    def parse_response(self):
        response_text = self.result.response.text
        self.get_logger().info(f'Raw response:\n{response_text}')

        if not response_text or response_text.strip() == "{}":
            self.get_logger().error('Empty or invalid response.')
            self.transition_to('DONE')
            return

        try:
            start = response_text.find('{')
            brace_count = 0
            for i in range(start, len(response_text)):
                if response_text[i] == '{':
                    brace_count += 1
                elif response_text[i] == '}':
                    brace_count -= 1
                    if brace_count == 0:
                        response_text = response_text[start:i + 1]
                        break

            data = json.loads(response_text)
            self.intention = data.get('intention', '')

            if not self.intention:
                self.get_logger().error('No intention found.')
                self.transition_to('DONE')
                return

            self.get_logger().info(f'Extracted intention: {self.intention}')
            self.transition_to('SAY_RESULT')

        except Exception as e:
            self.get_logger().error(f'Error parsing JSON: {e}')
            self.transition_to('DONE')

    def say_intention(self):
        goal = TTS.Goal()
        goal.text = f'Quieres beber: {self.intention}'
        self.get_logger().info(f'Speaking: "{goal.text}"')
        self.tts_client.send_goal_async(goal).add_done_callback(self.tts_result_callback)

    def tts_result_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('TTS goal was rejected.')
            self.transition_to('DONE')
            return
        goal_handle.get_result_async().add_done_callback(lambda _: self.transition_to('DONE'))

    def load_text_file(self, path):
        try:
            with open(path, 'r') as f:
                return f.read()
        except Exception as e:
            self.get_logger().error(f'Failed to read {path}: {e}')
            return ''

    def swap_placeholders(self, text, elements):

        for elem in elements:
            pos = text.find(self.placeholder)
            if pos != -1:
                text = text[:pos] + elem + text[pos + len(self.placeholder):]
            else:
                self.get_logger().warn(f'Placeholder "{self.placeholder}" not found.')
        return text


def main(args=None):
    rclpy.init(args=args)
    node = GenerateResponseNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
