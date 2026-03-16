import rclpy
from rclpy.node import Node
from hri_client.hri_client import HRIClient
from enum import Enum, auto

class State(Enum):
    INIT = auto()
    WAITING_INTRO = auto()
    WAITING_LISTENING = auto()
    WAITING_ECHO_INTRO = auto()
    WAITING_ECHO = auto()
    DONE = auto()

class HRIExampleClient(Node):

    def __init__(self):
        super().__init__('hri_example_client_node')
        self.hri_client = HRIClient(self)
        
        if not self.hri_client.wait_for_services(10.0):
            self.get_logger().error("Servicios no disponibles.")

        self.get_logger().info("✅ STT and TTS clients ready to use.")
        
        self.state = State.INIT
        self.transcribed_text = ""
        
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        if self.state == State.INIT:
            self.get_logger().info("🤖 Iniciando demostración de HRI...")
            self.hri_client.start_speaking("Hola. Vamos a probar el reconocimiento de voz y la síntesis de voz. Habla ahora.")
            self.state = State.WAITING_INTRO

        elif self.state == State.WAITING_INTRO:
            if self.hri_client.is_speaking_done():
                if self.hri_client.get_speaking_result():
                    self.get_logger().info("✅ TTS ejecutado correctamente")
                else:
                    self.get_logger().error("❌ Error en TTS")

                self.get_logger().info("🎤 Iniciando reconocimiento de voz (STT)...")
                self.hri_client.start_listen()
                self.state = State.WAITING_LISTENING

        elif self.state == State.WAITING_LISTENING:
            if self.hri_client.is_listen_done():
                self.transcribed_text = self.hri_client.get_listened_text()
                if not self.transcribed_text:
                    self.get_logger().error("❌ Error en STT")
                    self.state = State.DONE
                    return

                self.get_logger().info(f"📝 Transcripción obtenida: {self.transcribed_text}")
                self.hri_client.start_speaking("Ahora voy a repetir lo que has dicho")
                self.state = State.WAITING_ECHO_INTRO

        elif self.state == State.WAITING_ECHO_INTRO:
            if self.hri_client.is_speaking_done():
                self.get_logger().info("🔊 Enviando texto a TTS para reproducción...")
                self.hri_client.start_speaking(self.transcribed_text)
                self.state = State.WAITING_ECHO

        elif self.state == State.WAITING_ECHO:
            if self.hri_client.is_speaking_done():
                if self.hri_client.get_speaking_result():
                    self.get_logger().info("✅ TTS ejecutado correctamente")
                else:
                    self.get_logger().error("❌ Error en TTS")
                self.state = State.DONE

        elif self.state == State.DONE:
            self.get_logger().info("🎉 Demostración finalizada.")
            self.timer.cancel() # Detenemos el ciclo de control
            
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = HRIExampleClient()
    
    try:
        rclpy.spin(node) # Esto bloqueará el main procesando callbacks
    except SystemExit:
        pass # Salida limpia cuando State == DONE
    except KeyboardInterrupt:
        pass # Salida limpia con Ctrl+C

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
