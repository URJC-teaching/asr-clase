import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from simple_hri_interfaces.srv import Speech
from simple_hri_interfaces.srv import YesNo
from enum import Enum, auto

import time


class State(Enum):
    INIT = auto()
    WAITING_INTRO = auto()
    WAITING_INTRO_DELAY = auto()
    WAITING_USER_RESPONSE = auto()
    WAITING_YESNO = auto()
    WAITING_ECHO_DELAY = auto()
    WAITING_ECHO = auto()
    DONE = auto()


class HRIExample3(Node):

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

        # Extract client
        self.extract_client = self.create_client(YesNo, '/yesno_service')
        while not self.extract_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/yesno_service no disponible, esperando...')

        self.get_logger().info("✅ Clientes YesNo, STT y TTS listos para usar.")
        
        self.state = State.INIT
        self.user_response = ""
        self.current_future = None
        self.sleep_until = 0.0
        self.phrase_to_speak = ""
        
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
            self.get_logger().info("🤖 Iniciando demostración de HRI con YesNo...")

            tts_req = Speech.Request()
            tts_req.text = "¿Estás bien?"
            self.current_future = self.tts_client.call_async(tts_req)
            self.state = State.WAITING_INTRO

        elif self.state == State.WAITING_INTRO:
            if self.current_future and self.current_future.done():
                tts_response = self.current_future.result()
                if tts_response.success:
                    self.get_logger().info("✅ TTS ejecutado correctamente")
                else:
                    self.get_logger().error(f"❌ Error en TTS: {tts_response.debug}")

                # Le damos tiempo para que termine de hablar antes de encender STT
                self.set_sleep(3.0) 
                self.state = State.WAITING_INTRO_DELAY

        elif self.state == State.WAITING_INTRO_DELAY:
            self.get_logger().info("🎤 Iniciando reconocimiento de voz (STT)...")
            stt_req = SetBool.Request()
            stt_req.data = True
            self.current_future = None
            self.current_future = self.stt_client.call_async(stt_req)
            self.state = State.WAITING_USER_RESPONSE

        elif self.state == State.WAITING_USER_RESPONSE:
            if self.current_future and self.current_future.done():
                stt_response = self.current_future.result()
                if not stt_response.success:
                    self.get_logger().error(f"❌ Error en STT: {stt_response.message}")
                    self.user_response = ""
                else:
                    self.user_response = stt_response.message
                    self.get_logger().info(f"📝 Transcripción obtenida: {self.user_response}")

                self.get_logger().info("🔍 Enviando texto al servicio YesNo...")
                ext_req = YesNo.Request()
                ext_req.text = self.user_response
                self.current_future = None
                self.current_future = self.extract_client.call_async(ext_req)
                self.state = State.WAITING_YESNO

        elif self.state == State.WAITING_YESNO:
            if self.current_future and self.current_future.done():
                extract_response = self.current_future.result()
                extracted_text = extract_response.result
                self.get_logger().info(f"📝 Respuesta obtenida: {extracted_text}")

                if extracted_text and extracted_text != "ERROR":
                    self.get_logger().info(f"✅ Respuesta del servicio YesNo: {extracted_text}")
                    if extracted_text.lower() == 'yes':
                        self.phrase_to_speak = "He entendido: sí"
                        self.set_sleep(2.0)
                        self.state = State.WAITING_ECHO_DELAY
                    elif extracted_text.lower() == 'no':
                        self.phrase_to_speak = "He entendido: no"
                        self.set_sleep(2.0)
                        self.state = State.WAITING_ECHO_DELAY
                    else:
                        self.state = State.DONE
                else:
                    self.get_logger().error(f"❌ Error en YesNo: {extract_response.message}")
                    self.state = State.DONE


        elif self.state == State.WAITING_ECHO_DELAY:
            tts_req = Speech.Request()
            tts_req.text = self.phrase_to_speak
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
                
                self.state = State.DONE

        elif self.state == State.DONE:
            self.get_logger().info("🎉 Demostración finalizada.")
            self.timer.cancel()
            rclpy.shutdown()
            return


def main(args=None):
    rclpy.init(args=args)
    # Maintain original class name HRIExample2 to keep it working if they relied on it
    node = HRIExample3()
    
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
