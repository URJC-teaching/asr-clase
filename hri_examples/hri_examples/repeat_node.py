import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from audio_common_msgs.action import TTS
from whisper_msgs.action import STT


class RepeatNode(Node):

    def __init__(self):
        super().__init__('repeat_node')

        self.tts_client = ActionClient(self, TTS, 'say')
        self.stt_client = ActionClient(self, STT, 'whisper/listen')

        self.state = 'INIT'
        self.transcribed_text = ''

        self.get_logger().info('Initializing RepeatNode...')
        self.start()

    def start(self):
        self.get_logger().info('Waiting for action servers...')
        if not self.tts_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('TTS server not available.')
            return
        if not self.stt_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('STT server not available.')
            return

        self.transition_to('INIT')

    def transition_to(self, new_state):
        self.get_logger().info(f'Transitioning to state: {new_state}')
        self.state = new_state

        if self.state == 'INIT':
            self.say_prompt()
        elif self.state == 'LISTEN':
            self.listen()
        elif self.state == 'REPEAT':
            self.say_transcription()
        elif self.state == 'DONE':
            self.get_logger().info('Interaction finished.')
            rclpy.shutdown()

    def say_prompt(self):
        goal = TTS.Goal()
        goal.text = "¿Qué quieres que repita?"
        self.get_logger().info(f'Speaking: "{goal.text}"')
        self.tts_client.send_goal_async(goal).add_done_callback(self.on_prompt_sent)

    def on_prompt_sent(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('TTS prompt goal was rejected.')
            self.transition_to('DONE')
            return
        goal_handle.get_result_async().add_done_callback(lambda f: self.transition_to('LISTEN'))

    def listen(self):
        goal = STT.Goal()
        self.get_logger().info('Listening for speech...')
        self.stt_client.send_goal_async(goal).add_done_callback(self.on_listen_sent)

    def on_listen_sent(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('STT goal was rejected.')
            self.transition_to('DONE')
            return
        goal_handle.get_result_async().add_done_callback(self.on_listen_result)

    def on_listen_result(self, future):
        self.get_logger().info('Speech recognition completed.')
        result = future.result().result
        self.transcribed_text = result.transcription.text if hasattr(result.transcription, 'text') else ''
        self.get_logger().info(f'Recognized: "{self.transcribed_text}"')

        if not self.transcribed_text.strip():
            self.get_logger().warn('No speech recognized.')
            self.transition_to('DONE')
        else:
            self.transition_to('REPEAT')

    def say_transcription(self):
        goal = TTS.Goal()
        goal.text = self.transcribed_text
        self.get_logger().info(f'Repeating: "{goal.text}"')
        self.tts_client.send_goal_async(goal).add_done_callback(self.on_repeat_sent)

    def on_repeat_sent(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('TTS repeat goal was rejected.')
            self.transition_to('DONE')
            return
        goal_handle.get_result_async().add_done_callback(lambda f: self.transition_to('DONE'))


def main(args=None):
    rclpy.init(args=args)
    node = RepeatNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
