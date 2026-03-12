import rclpy
from rclpy.node import Node
from hri_client.hri_client import HRIClient


class HRIExample2Client(Node):

    def __init__(self):
        super().__init__('hri_example2_client_node')
        self.hri_client = HRIClient()
    
        if not self.hri_client.wait_for_services(10.0):
            self.get_logger().info('Servicios no disponibles, esperando...')

        self.get_logger().info("✅ Clientes Extract, STT y TTS listos para usar.")

    def call_tts(self, text, sleep_time=4.0):
        self.start_speaking(text)
        # Se requiere timeout_sec para evitar que el nodo se quede bloqueado de 
        # forma indefinida esperando eventos si no hay mensajes nuevos inmediatos.
        while rclpy.ok() and not self.is_speaking_done():
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if self.get_speaking_result():
            self.get_logger().info("✅ TTS ejecutado correctamente")
        else:
            self.get_logger().error("❌ Error en TTS")

    def call_stt(self):
        self.get_logger().info("🎤 Iniciando reconocimiento de voz (STT)...")
        self.hri_client.start_listen()
        # El timeout_sec permite que el ejecutor no se quede bloqueado si no hay
        # eventos inmediatamente, dando paso a evaluar la condición del bucle.
        while rclpy.ok() and not self.hri_client.is_listen_done():
            rclpy.spin_once(self.hri_client, timeout_sec=0.1)

        transcription = self.hri_client.get_listened_text()
        if not transcription:
            self.get_logger().error("❌ Error en STT")
            return ""

        self.get_logger().info(f"📝 Transcripción obtenida: {transcription}")
        return transcription
        
    def call_extract(self, text, interest):
        self.get_logger().info("🔍 Enviando texto al servicio Extract...")
        self.hri_client.start_extract(interest, text)
        # Spin_once con timeout asegura que podamos salir del bucle en cuanto .is_extract_done() 
        # devuelva True, en lugar de quedarnos bloqueados indefinidamente por falta de callbacks.
        while rclpy.ok() and not self.hri_client.is_extract_done():
            rclpy.spin_once(self.hri_client, timeout_sec=0.1)

        extracted_response = self.hri_client.get_extracted_info()
        self.get_logger().info(f"📝 Extracto obtenido: {extracted_response}")

        if extracted_response == "ERROR":
            self.get_logger().error("❌ Error en Extract")
            return None
        
        return extracted_response
    

    def order_to_string(self, order_list, prefix):
        n = len(order_list)
        phrase = prefix

        if order_list[0] == "NONE":
            phrase += "nada."
            return phrase
        elif len(order_list) == 1:
            phrase += order_list[0] + "."
            return phrase
        
        for i in range(n):
            if i == n - 1 and n > 1: # Último ítem
                phrase += "y " + order_list[i] + "."
            else:
                phrase += order_list[i] + ", "
        
        return phrase

    def run(self):
        self.get_logger().info("🤖 Iniciando demostración de HRI...")

        # TTS para pedir al usuario que hable
        self.call_tts("Hola. Vamos a probar la extracción de información. Imagina que soy un camarero y tú eres un cliente que va a hacer un pedido. ¿Qué te gustaría pedir de beber y de comer?", sleep_time=8.0)


        # STT para capturar la respuesta del usuario
        user_response = self.call_stt()

        if not user_response:
             user_response = ""

        # Llamadas a Extract para extraer ítems de interés
        extracted_text = self.call_extract(user_response, "bebida")
        if extracted_text:
            list_items = extracted_text.strip('\n').split(";")
            n = len(list_items)
            self.get_logger().info(f"✅ Se han extraído {n} ítems de interés.")
            phrase = self.order_to_string(list_items, "De beber, has pedido: ")
            self.call_tts(phrase, sleep_time=4.0)

        # Repetir para platos principales
        extracted_text = self.call_extract(user_response, "platos principales")
        if extracted_text:
            list_items = extracted_text.strip('\n').split(";")
            n = len(list_items)
            self.get_logger().info(f"✅ Se han extraído {n} ítems de interés.")
            phrase = self.order_to_string(list_items, "Y de comer, has pedido: ")
            self.call_tts(phrase, sleep_time=4.0)

        # Repetir para postres
        extracted_text = self.call_extract(user_response, "postres")
        if extracted_text:
            list_items = extracted_text.strip('\n').split(";")
            n = len(list_items)
            self.get_logger().info(f"✅ Se han extraído {n} ítems de interés.")
            phrase = self.order_to_string(list_items, "De postre, quieres: ")
            self.call_tts(phrase, sleep_time=4.0)


def main(args=None):
    rclpy.init(args=args)
    node = HRIExample2Client()
    node.run()
    node.hri_client.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
