import rclpy
from rclpy.node import Node
from hri_client.hri_client import HRIClient
from enum import Enum, auto


class State(Enum):
    INIT = auto()
    WAITING_INTRO = auto()
    WAITING_USER_RESPONSE = auto()
    WAITING_YESNO = auto()
    WAITING_ECHO = auto()
    DONE = auto()


class HRIExample3Client(Node):

    def __init__(self):
        super().__init__('hri_example3_client_node')
        self.hri_client = HRIClient(self)
    
        if not self.hri_client.wait_for_services(10.0):
            self.get_logger().info('Servicios no disponibles, esperando...')

        self.get_logger().info("✅ Clientes YesNo, STT y TTS listos para usar.")
        
        self.state = State.INIT
        self.user_response = ""
        
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        if self.state == State.INIT:
            self.get_logger().info("🤖 Iniciando demostración de HRI con YesNo...")
            self.hri_client.start_speaking("¿Estás bien?")
            self.state = State.WAITING_INTRO

        elif self.state == State.WAITING_INTRO:
            if self.hri_client.is_speaking_done():
                if self.hri_client.get_speaking_result():
                    self.get_logger().info("✅ TTS ejecutado correctamente")
                else:
                    self.get_logger().error("❌ Error en TTS")

                self.get_logger().info("🎤 Iniciando reconocimiento de voz (STT)...")
                self.hri_client.start_listen()
                self.state = State.WAITING_USER_RESPONSE

        elif self.state == State.WAITING_USER_RESPONSE:
            if self.hri_client.is_listen_done():
                self.user_response = self.hri_client.get_listened_text()
                if not self.user_response:
                    self.get_logger().error("❌ Error en STT")
                    self.user_response = ""
                else:
                    self.get_logger().info(f"📝 Transcripción obtenida: {self.user_response}")

                self.get_logger().info("🔍 Enviando texto al servicio YesNo...")
                self.hri_client.start_yesno(self.user_response)
                self.state = State.WAITING_YESNO

        elif self.state == State.WAITING_YESNO:
            if self.hri_client.is_yesno_done():
                extracted_response = self.hri_client.get_yesno_result()
                self.get_logger().info(f"📝 Respuesta obtenida: {extracted_response}")
                
                if extracted_response and extracted_response != "ERROR":
                    self.get_logger().info(f"✅ Respuesta del servicio YesNo: {extracted_response}")
                    if extracted_response.lower() == 'yes':
                        self.hri_client.start_speaking("He entendido: sí")
                        self.state = State.WAITING_ECHO
                    elif extracted_response.lower() == 'no':
                        self.hri_client.start_speaking("He entendido: no")
                        self.state = State.WAITING_ECHO
                    else:
                        self.state = State.DONE
                else:
                    self.get_logger().error("❌ Error en YesNo")
                    self.state = State.DONE

        elif self.state == State.WAITING_ECHO:
            if self.hri_client.is_speaking_done():
                if self.hri_client.get_speaking_result():
                    self.get_logger().info("✅ TTS ejecutado correctamente")
                else:
                    self.get_logger().error("❌ Error en TTS")
                self.state = State.DONE

        elif self.state == State.DONE:
            self.get_logger().info("🎉 Demostración finalizada.")
            self.timer.cancel()
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = HRIExample3Client()
    
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
