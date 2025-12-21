import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from simple_hri_interfaces.srv import Speech
from nao_lola_command_msgs.msg import ChestLed
from rclpy.action import ActionClient
from nao_pos_interfaces.action import PosPlay
from nao_led_interfaces.action import LedsPlay
from nao_led_interfaces.msg import LedIndexes, LedModes
from std_msgs.msg import ColorRGBA
import time

class NaoHRIExample(Node):

    def __init__(self):
        super().__init__('nao_hri_example_node')
    
        # STT client
        self.stt_client = self.create_client(SetBool, '/stt_service')
        while not self.stt_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/stt_service unavailable...')

        # TTS client
        self.tts_client = self.create_client(Speech, '/tts_service')
        while not self.tts_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/tts_service unavailable...')

        self.get_logger().info("✅ STT and TTS clients ready to use.")

        # Position action client
        self.pos_client = ActionClient(self, PosPlay, '/nao_pos_action')
        
        self.get_logger().info("✅ Position action client ready to use.")

        # Publisher to change chest led color
        self.chest_led_pub = self.create_publisher(ChestLed, '/effectors/chest_led', 10)

        self.leds_client = ActionClient(self, LedsPlay, '/leds_play')
        self.switch_off_all_leds()


    def run(self):

        self.get_logger().info("🤖 Iniciando demostración de HRI con Nao...")

        pos_goal = PosPlay.Goal()
        pos_goal.action_name = "hello"
        
        self.get_logger().info("⏳ Sending position goal...")
        send_goal_future = self.pos_client.send_goal_async(pos_goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected")
            return

        chest_led_msg = ChestLed()
        chest_led_msg.color.r = 1.0
        chest_led_msg.color.g = 0.0
        chest_led_msg.color.b = 0.0
        self.chest_led_pub.publish(chest_led_msg)

        tts_req = Speech.Request()
        tts_req.text = "Hola, soy Nao. Vamos a probar el reconocimiento de voz y la síntesis de voz. Habla cuando la luz de mi pecho esté azul."
        tts_future = self.tts_client.call_async(tts_req)
        rclpy.spin_until_future_complete(self, tts_future)
        tts_response = tts_future.result()

        time.sleep(8.0)
     
        if tts_response.success:
            self.get_logger().info("✅ TTS ejecutado correctamente")
        else:
            self.get_logger().error(f"❌ Error en TTS: {tts_response.debug}")

        chest_led_msg.color.r = 0.0
        chest_led_msg.color.g = 0.0
        chest_led_msg.color.b = 1.0
        self.chest_led_pub.publish(chest_led_msg)

        self.get_logger().info("🎤 Iniciando reconocimiento de voz (STT)...")
        stt_req = SetBool.Request()
        stt_req.data = True  # Indica al servicio que inicie grabación

        stt_future = self.stt_client.call_async(stt_req)
        rclpy.spin_until_future_complete(self, stt_future)
        stt_response = stt_future.result()

        if not stt_response.success:
            self.get_logger().error(f"❌ Error en STT: {stt_response.message}")
            return

        transcribed_text = stt_response.message
        self.get_logger().info(f"📝 Transcripción obtenida: {transcribed_text}")

        chest_led_msg.color.r = 1.0
        chest_led_msg.color.g = 0.0
        chest_led_msg.color.b = 0.0
        self.chest_led_pub.publish(chest_led_msg)
        tts_req.text = "Ahora voy a repetir lo que has dicho"
        tts_future = self.tts_client.call_async(tts_req)
        rclpy.spin_until_future_complete(self, tts_future)
        tts_response = tts_future.result()
        time.sleep(4.0)


        chest_led_msg.color.r = 0.0
        chest_led_msg.color.g = 1.0
        chest_led_msg.color.b = 0.0
        self.chest_led_pub.publish(chest_led_msg)

        self.get_logger().info("🔊 Enviando texto a TTS para reproducción...")
        tts_req.text = transcribed_text

        tts_future = self.tts_client.call_async(tts_req)
        rclpy.spin_until_future_complete(self, tts_future)
        tts_response = tts_future.result()
        time.sleep(5.0)

        if tts_response.success:
            self.get_logger().info("✅ TTS ejecutado correctamente")
        else:
            self.get_logger().error(f"❌ Error en TTS: {tts_response.debug}")
        
        chest_led_msg.color.r = 0.0
        chest_led_msg.color.g = 0.0
        chest_led_msg.color.b = 0.0
        self.chest_led_pub.publish(chest_led_msg)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Success: {result.success}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted :)')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def switch_off_all_leds(self):
        N_COLORS = 8
        N_INTENS = 12

        def black_colors(n):
            # crea instancias independientes y usa floats
            return [ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0) for _ in range(n)]

        goal_msg = LedsPlay.Goal()
        goal_msg.leds = [LedIndexes.REYE, LedIndexes.LEYE]
        goal_msg.mode = LedModes.STEADY
        goal_msg.frequency = 0.0
        goal_msg.colors = black_colors(N_COLORS)
        goal_msg.intensities = [0.0] * N_INTENS
        goal_msg.duration = 0.0
        send_goal_future = self.leds_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

        goal_msg = LedsPlay.Goal()
        goal_msg.leds = [LedIndexes.CHEST]
        goal_msg.mode = LedModes.STEADY
        goal_msg.frequency = 0.0
        goal_msg.colors = black_colors(N_COLORS)
        goal_msg.intensities = [0.0] * N_INTENS
        goal_msg.duration = 0.0
        send_goal_future = self.leds_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

        goal_msg = LedsPlay.Goal()
        goal_msg.leds = [LedIndexes.REAR, LedIndexes.LEAR]
        goal_msg.mode = LedModes.STEADY
        goal_msg.frequency = 0.0
        goal_msg.colors = black_colors(N_COLORS)
        goal_msg.intensities = [0.0] * N_INTENS
        goal_msg.duration = 0.0
        send_goal_future = self.leds_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)



def main(args=None):
    rclpy.init(args=args)
    node = NaoHRIExample()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
