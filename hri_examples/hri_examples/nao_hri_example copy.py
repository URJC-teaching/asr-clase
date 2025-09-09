import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from hni_interfaces.srv import TextToSpeech
from nao_lola_command_msgs.msg import ChestLed
import time


class NaoHRIExample(Node):

    def __init__(self):
        super().__init__('nao_hri_example_node')
    
        # STT client
        self.stt_client = self.create_client(SetBool, '/stt_service')
        while not self.stt_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/stt_service unavailable...')

        # TTS client
        self.tts_client = self.create_client(TextToSpeech, '/tts_service')
        while not self.tts_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/tts_service unavailable...')

        self.get_logger().info("✅ STT and TTS clients ready to use.")

        # Publisher to change chest led color
        self.chest_led_pub = self.create_publisher(ChestLed, '/effectors/chest_led', 10)

    def run(self):

        led = ChestLed()
        led.red, led.green, led.blue = 1.0, 0.0, 0.0
        self.chest_led_pub.publish(led)

        tts_req = TextToSpeech.Request()
        tts_req.text = "Hola, soy Nao. Habla cuando la luz de mi pecho esté azul."
        future = self.tts_client.call_async(tts_req)
        rclpy.spin_until_future_complete(self, future)

        time.sleep(5)

        led.red, led.green, led.blue = 0.0, 0.0, 1.0
        self.chest_led_pub.publish(led)

        stt_req = SetBool.Request()
        stt_req.data = True
        stt_future = self.stt_client.call_async(stt_req)
        rclpy.spin_until_future_complete(self, stt_future)
        stt_response = stt_future.result()

        if not stt_response.success:
            self.get_logger().error(f"❌ Error en STT: {stt_response.message}")
            return

        transcribed = stt_response.message
        self.get_logger().info(f"📝 Texto captado: {transcribed}")

        led.red, led.green, led.blue = 0.0, 1.0, 0.0
        self.chest_led_pub.publish(led)

        tts_req.text = transcribed
        future = self.tts_client.call_async(tts_req)
        rclpy.spin_until_future_complete(self, future)

        led.red = led.green = led.blue = 0.0
        self.chest_led_pub.publish(led)


def main(args=None):
    rclpy.init(args=args)
    node = NaoHRIExample()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
