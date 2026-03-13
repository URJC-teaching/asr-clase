import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import SetBool
from simple_hri_interfaces.srv import Speech
from enum import Enum, auto

import time


class State(Enum):
    INIT = auto()
    WAITING_INTRO = auto()
    WAITING_DELAY = auto()
    WAITING_LISTENING = auto()
    WAITING_ECHO_INTRO = auto()
    WAITING_ECHO_DELAY = auto()
    WAITING_ECHO = auto()
    DONE = auto()


class HRIExample(Node):

    def __init__(self):
        super().__init__('hri_example_node')
    
        # STT client
        self.stt_client = self.create_client(SetBool, '/stt_service')
        while not self.stt_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/stt_service unavailable...')

        # TTS client
        self.tts_client = self.create_client(Speech, '/tts_service')
        while not self.tts_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/tts_service unavailable...')

        self.get_logger().info("✅ STT and TTS clients ready to use.")
        
        self.state = State.INIT
        self.transcribed_text = ""
        self.current_future = None
        self.sleep_until = 0.0
        
        # Ejecutamos el control_loop() cada 0.1 segundos (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

    def is_sleeping(self):
        return time.time() < self.sleep_until

    def set_sleep(self, seconds):
        self.sleep_until = time.time() + seconds

    def control_loop(self):
        if self.is_sleeping():
            return

        if self.state == State.INIT:
            self.get_logger().info("🤖 Iniciando demostración de HRI...")

            tts_req = Speech.Request()
            tts_req.text = "Hola. Vamos a probar el reconocimiento de voz y la síntesis de voz. Habla ahora."
            self.current_future = self.tts_client.call_async(tts_req)
            self.state = State.WAITING_INTRO

        elif self.state == State.WAITING_INTRO:
            if self.current_future and self.current_future.done():
                tts_response = self.current_future.result()
                if tts_response.success:
                    self.get_logger().info("✅ TTS ejecutado correctamente")
                else:
                    self.get_logger().error(f"❌ Error en TTS: {tts_response.debug}")

                # La intro ha terminado en TTS, pero internamente el audio todavía podría  
                # estar sonando. Ponemos un delay no bloqueante y vamos a un estado intermedio
                self.set_sleep(8.0) 
                self.state = State.WAITING_DELAY

        elif self.state == State.WAITING_DELAY:
            # Una vez pasados los 8 segundos del sleep, saltamos aquí.
            self.get_logger().info("🎤 Iniciando reconocimiento de voz (STT)...")
            stt_req = SetBool.Request()
            stt_req.data = True  # Indica al servicio que inicie grabación
            self.current_future = None
            self.current_future = self.stt_client.call_async(stt_req)
            self.state = State.WAITING_LISTENING

        elif self.state == State.WAITING_LISTENING:
            if self.current_future and self.current_future.done():
                stt_response = self.current_future.result()
                if not stt_response.success:
                    self.get_logger().error(f"❌ Error en STT: {stt_response.message}")
                    self.state = State.DONE
                    return

                self.transcribed_text = stt_response.message
                self.get_logger().info(f"📝 Transcripción obtenida: {self.transcribed_text}")

                tts_req = Speech.Request()
                tts_req.text = "Ahora voy a repetir lo que has dicho"
                self.current_future = None
                self.current_future = self.tts_client.call_async(tts_req)
                self.state = State.WAITING_ECHO_INTRO

        elif self.state == State.WAITING_ECHO_INTRO:
            if self.current_future and self.current_future.done():
                tts_response = self.current_future.result()
                self.set_sleep(4.0)
                self.state = State.WAITING_ECHO_DELAY

        elif self.state == State.WAITING_ECHO_DELAY:
            # Una vez pasados los 4 segundos del sleep en WAITING_ECHO_INTRO
            self.get_logger().info("🔊 Enviando texto a TTS para reproducción...")
            tts_req = Speech.Request()
            tts_req.text = self.transcribed_text
            self.current_future = None
            self.current_future = self.tts_client.call_async(tts_req)
            self.state = State.WAITING_ECHO

        elif self.state == State.WAITING_ECHO:
            if self.current_future and self.current_future.done():
                tts_response = self.current_future.result()
                if tts_response.success:
                    self.get_logger().info("✅ TTS ejecutado correctamente")
                else:
                    self.get_logger().error(f"❌ Error en TTS: {tts_response.debug}")
                
                # self.set_sleep(5.0)
                self.state = State.DONE

        elif self.state == State.DONE:
            self.get_logger().info("🎉 Demostración finalizada.")
            self.timer.cancel()
            
            # Since rclpy.spin catches SystemExit softly sometimes when inside a Timer, 
            # the safest way to shut down a ROS2 node from within a callback is:
            rclpy.shutdown()
            return


def main(args=None):
    rclpy.init(args=args)
    node = HRIExample()
    
    try:
        rclpy.spin(node)
    except Exception:
        pass

    try:
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
