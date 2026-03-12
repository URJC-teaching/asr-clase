import time
import rclpy
from rclpy.node import Node
from hri_client.hri_client import HRIClient


class HRIExample3Client(Node):

    def __init__(self):
        super().__init__('hri_example3_client_node')
        self.hri_client = HRIClient()
    
        if not self.hri_client.wait_for_services(10.0):
            self.get_logger().info('Servicios no disponibles, esperando...')

        self.get_logger().info("✅ Clientes YesNo, STT y TTS listos para usar.")

    def call_tts(self, text, sleep_time=4.0):
        self.hri_client.start_speaking(text)
        # Se requiere timeout_sec para evitar que el nodo se quede bloqueado de 
        # forma indefinida esperando eventos si no hay mensajes nuevos inmediatos.
        while rclpy.ok() and not self.hri_client.is_speaking_done():
            rclpy.spin_once(self.hri_client, timeout_sec=0.1)
            
        time.sleep(sleep_time)

        if self.hri_client.get_speaking_result():
            self.get_logger().info("✅ TTS ejecutado correctamente")
        else:
            self.get_logger().error("❌ Error en TTS")

    def call_stt(self):
        self.get_logger().info("🎤 Iniciando reconocimiento de voz (STT)...")
        self.hri_client.start_listen()
        while rclpy.ok() and not self.hri_client.is_listen_done():
            rclpy.spin_once(self.hri_client, timeout_sec=0.1)

        transcription = self.hri_client.get_listened_text()
        if not transcription:
            self.get_logger().error("❌ Error en STT")
            return ""

        self.get_logger().info(f"📝 Transcripción obtenida: {transcription}")
        return transcription
        
    def call_yesno(self, text):
        self.get_logger().info("🔍 Enviando texto al servicio YesNo...")
        self.hri_client.start_yesno(text)
        # Timeout necesario para que rclpy libere el hilo (thread lock) con frecuencia 
        # y evalúe la condición is_yesno_done() basada en el timeout del Action/Service.
        while rclpy.ok() and not self.hri_client.is_yesno_done():
            rclpy.spin_once(self.hri_client, timeout_sec=0.1)

        extracted_response = self.hri_client.get_yesno_result()
        self.get_logger().info(f"📝 Respuesta obtenida: {extracted_response}")

        if extracted_response == "ERROR":
            self.get_logger().error("❌ Error en YesNo")
            return None
        
        return extracted_response
    

    def run(self):
        self.get_logger().info("🤖 Iniciando demostración de HRI...")

        # TTS para pedir al usuario que hable
        self.call_tts("¿Estás bien?", sleep_time=3.0)

        # STT para capturar la respuesta del usuario
        user_response = self.call_stt()

        if not user_response:
             user_response = ""

        # Llamadas a YesNo para extraer ítems de interés
        extracted_text = self.call_yesno(user_response)
        if extracted_text:
            self.get_logger().info(f"✅ Respuesta del servicio YesNo: {extracted_text}")

            if extracted_text.lower() == 'yes':
                self.call_tts("He entendido: sí", sleep_time=4.0)
            elif extracted_text.lower() == 'no':
                self.call_tts("He entendido: no", sleep_time=4.0)


def main(args=None):
    rclpy.init(args=args)
    node = HRIExample3Client()
    node.run()
    node.hri_client.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
