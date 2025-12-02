import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from hni_interfaces.srv import TextToSpeech

from simple_hri_interfaces.srv import Extract
import time

class HRIExample2(Node):

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

        # Extract client
        self.extract_client = self.create_client(Extract, '/extract_service')
        while not self.extract_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/extract_service no disponible, esperando...')

        self.get_logger().info("✅ Clientes Extract, STT y TTS listos para usar.")

    def call_tts(self, text, sleep_time=4.0):
        tts_req = TextToSpeech.Request()
        tts_req.text = text
        tts_future = self.tts_client.call_async(tts_req)
        rclpy.spin_until_future_complete(self, tts_future)
        tts_response = tts_future.result()
        time.sleep(sleep_time)

        if tts_response.success:
            self.get_logger().info("✅ TTS ejecutado correctamente")
        else:
            self.get_logger().error(f"❌ Error en TTS: {tts_response.debug}")

    def call_stt(self):
        self.get_logger().info("🎤 Iniciando reconocimiento de voz (STT)...")
        stt_req = SetBool.Request()
        stt_req.data = True  # Indica al servicio que inicie grabación

        stt_future = self.stt_client.call_async(stt_req)
        rclpy.spin_until_future_complete(self, stt_future)
        stt_response = stt_future.result()

        if not stt_response.success:
            self.get_logger().error(f"❌ Error en STT: {stt_response.message}")
            return None

        self.get_logger().info(f"📝 Transcripción obtenida: {stt_response.message}")
        return stt_response.message
        
    def call_extract(self, text, interest):
        self.get_logger().info("🔍 Enviando texto al servicio Extract...")
        extract_req = Extract.Request()
        extract_req.text = text
        extract_req.interest = interest
        extract_future = self.extract_client.call_async(extract_req)
        rclpy.spin_until_future_complete(self, extract_future)
        extract_response = extract_future.result()

        self.get_logger().info(f"📝 Extracto obtenido: {extract_response.result}")

        if extract_response.result == "ERROR":
            self.get_logger().error(f"❌ Error en Extract: {extract_response.message}")
            return None
        
        return extract_response.result
    

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

        # Llamadas a Extract para extraer ítems de interés
        extracted_text = self.call_extract(user_response, "bebida")
        list_items = extracted_text.strip('\n').split(";")
        n = len(list_items)
        self.get_logger().info(f"✅ Se han extraído {n} ítems de interés.")

        # Formar la frase para TTS
        phrase = self.order_to_string(list_items, "De beber, has pedido: ")
        
        # TTS para comunicar el pedido al usuario
        self.call_tts(phrase, sleep_time=4.0)

        # Repetir para platos principales
        extracted_text = self.call_extract(user_response, "platos principales")
        list_items = extracted_text.strip('\n').split(";")
        n = len(list_items)
        self.get_logger().info(f"✅ Se han extraído {n} ítems de interés.")

        # Formar la frase para TTS
        phrase = self.order_to_string(list_items, "Y de comer, has pedido: ")
   
        # TTS para comunicar el pedido al usuario
        self.call_tts(phrase, sleep_time=4.0)

        # Repetir para postres
        extracted_text = self.call_extract(user_response, "postres")
        list_items = extracted_text.strip('\n').split(";")
        n = len(list_items)
        self.get_logger().info(f"✅ Se han extraído {n} ítems de interés.")
        
        # Formar la frase para TTS
        phrase = self.order_to_string(list_items, "De postre, quieres: ")
   
        # TTS para comunicar el pedido al usuario
        self.call_tts(phrase, sleep_time=4.0)


def main(args=None):
    rclpy.init(args=args)
    node = HRIExample2()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
