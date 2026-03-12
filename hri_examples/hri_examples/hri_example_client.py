import rclpy
from rclpy.node import Node
from hri_client.hri_client import HRIClient


class HRIExampleClient(Node):

    def __init__(self):
        super().__init__('hri_example_client_node')
        self.hri_client = HRIClient()
        
        if not self.hri_client.wait_for_services(10.0):
            self.get_logger().error("Servicios no disponibles.")

        self.get_logger().info("✅ STT and TTS clients ready to use.")

    def run(self):
        self.get_logger().info("🤖 Iniciando demostración de HRI...")

        self.hri_client.start_speaking("Hola. Vamos a probar el reconocimiento de voz y la síntesis de voz. Habla ahora.")
        # Se requiere timeout_sec para evitar que el nodo se quede bloqueado de 
        # forma indefinida esperando mensajes de la red en la cola del ejecutor.
        while rclpy.ok() and not self.hri_client.is_speaking_done():
            rclpy.spin_once(self.hri_client, timeout_sec=0.1)
    
        if self.hri_client.get_speaking_result():
            self.get_logger().info("✅ TTS ejecutado correctamente")
        else:
            self.get_logger().error("❌ Error en TTS")

        self.get_logger().info("🎤 Iniciando reconocimiento de voz (STT)...")
        self.hri_client.start_listen()
        while rclpy.ok() and not self.hri_client.is_listen_done():
            rclpy.spin_once(self.hri_client, timeout_sec=0.1)

        transcribed_text = self.hri_client.get_listened_text()
        if not transcribed_text:
            self.get_logger().error("❌ Error en STT")
            return

        self.get_logger().info(f"📝 Transcripción obtenida: {transcribed_text}")

        self.hri_client.start_speaking("Ahora voy a repetir lo que has dicho")
        while rclpy.ok() and not self.hri_client.is_speaking_done():
            rclpy.spin_once(self.hri_client, timeout_sec=0.1)

        self.get_logger().info("🔊 Enviando texto a TTS para reproducción...")
        self.hri_client.start_speaking(transcribed_text)
        while rclpy.ok() and not self.hri_client.is_speaking_done():
            rclpy.spin_once(self.hri_client, timeout_sec=0.1)

        if self.hri_client.get_speaking_result():
            self.get_logger().info("✅ TTS ejecutado correctamente")
        else:
            self.get_logger().error("❌ Error en TTS")


def main(args=None):
    rclpy.init(args=args)
    node = HRIExampleClient()
    node.run()
    node.hri_client.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
