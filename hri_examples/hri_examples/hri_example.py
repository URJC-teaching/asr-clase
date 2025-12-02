import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from hni_interfaces.srv import TextToSpeech

import time

class HRIExample(Node):

    def __init__(self):
        super().__init__('hri_example_node')
    
        # STT client
        self.stt_client = self.create_client(SetBool, '/stt_service')
        while not self.stt_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/stt_service unavailable...')

        # TTS client
        self.tts_client = self.create_client(TextToSpeech, '/tts_service')
        while not self.tts_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/tts_service unavailable...')

        self.get_logger().info("✅ STT and TTS clients ready to use.")


    def run(self):

        self.get_logger().info("🤖 Iniciando demostración de HRI...")


        tts_req = TextToSpeech.Request()
        tts_req.text = "Hola. Vamos a probar el reconocimiento de voz y la síntesis de voz. Habla ahora."
        tts_future = self.tts_client.call_async(tts_req)
        rclpy.spin_until_future_complete(self, tts_future)
        tts_response = tts_future.result()

        time.sleep(8.0)
     
        if tts_response.success:
            self.get_logger().info("✅ TTS ejecutado correctamente")
        else:
            self.get_logger().error(f"❌ Error en TTS: {tts_response.debug}")


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


        tts_req.text = "Ahora voy a repetir lo que has dicho"
        tts_future = self.tts_client.call_async(tts_req)
        rclpy.spin_until_future_complete(self, tts_future)
        tts_response = tts_future.result()
        time.sleep(4.0)


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

    # def get_result_callback(self, future):
    #     result = future.result().result
    #     self.get_logger().info(f'Success: {result.success}')

    # def goal_response_callback(self, future):
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         self.get_logger().info('Goal rejected :(')
    #         rclpy.shutdown()
    #         return

    #     self.get_logger().info('Goal accepted :)')

    #     get_result_future = goal_handle.get_result_async()
    #     get_result_future.add_done_callback(self.get_result_callback)




def main(args=None):
    rclpy.init(args=args)
    node = HRIExample()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
