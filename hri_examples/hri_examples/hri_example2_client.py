import rclpy
from rclpy.node import Node
from hri_client.hri_client import HRIClient
from enum import Enum, auto


class State(Enum):
    INIT = auto()
    WAITING_INTRO = auto()
    WAITING_USER_RESPONSE = auto()
    WAITING_EXTRACT_DRINK = auto()
    WAITING_ECHO_DRINK = auto()
    WAITING_EXTRACT_FOOD = auto()
    WAITING_ECHO_FOOD = auto()
    WAITING_EXTRACT_DESSERT = auto()
    WAITING_ECHO_DESSERT = auto()
    DONE = auto()


class HRIExample2Client(Node):

    def __init__(self):
        super().__init__('hri_example2_client_node')
        self.hri_client = HRIClient(self)
    
        if not self.hri_client.wait_for_services(10.0):
            self.get_logger().info('Servicios no disponibles, esperando...')

        self.get_logger().info("✅ Clientes Extract, STT y TTS listos para usar.")
        
        self.state = State.INIT
        self.user_response = ""
        
        # Ejecutamos el control_loop() cada 0.1 segundos (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

    def order_to_string(self, order_list, prefix):
        n = len(order_list)
        phrase = prefix

        if not order_list:
            return phrase + "nada."

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

    def control_loop(self):
        if self.state == State.INIT:
            self.get_logger().info("🤖 Iniciando demostración de HRI con Extract...")
            self.hri_client.start_speaking(
                "Hola. Vamos a probar la extracción de información. Imagina que soy un camarero "
                "y tú eres un cliente que va a hacer un pedido. ¿Qué te gustaría pedir de beber y de comer?"
            )
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

                self.get_logger().info("🔍 Enviando texto al servicio Extract (bebida)...")
                self.hri_client.start_extract("bebida", self.user_response)
                self.state = State.WAITING_EXTRACT_DRINK

        elif self.state == State.WAITING_EXTRACT_DRINK:
            if self.hri_client.is_extract_done():
                extracted_text = self.hri_client.get_extracted_info()
                self.get_logger().info(f"📝 Extracto obtenido (bebida): {extracted_text}")

                if extracted_text and extracted_text != "ERROR":
                    list_items = extracted_text.strip('\n').split(";")
                    n = len(list_items)
                    self.get_logger().info(f"✅ Se han extraído {n} ítems de interés.")
                    phrase = self.order_to_string(list_items, "De beber, has pedido: ")
                    self.hri_client.start_speaking(phrase)
                    self.state = State.WAITING_ECHO_DRINK
                else:
                    # Si falla, pasamos directo a procesar la comida
                    self.get_logger().info("🔍 Enviando texto al servicio Extract (platos principales)...")
                    self.hri_client.start_extract("platos principales", self.user_response)
                    self.state = State.WAITING_EXTRACT_FOOD

        elif self.state == State.WAITING_ECHO_DRINK:
            if self.hri_client.is_speaking_done():
                # Pasamos a procesar platos principales
                self.get_logger().info("🔍 Enviando texto al servicio Extract (platos principales)...")
                self.hri_client.start_extract("platos principales", self.user_response)
                self.state = State.WAITING_EXTRACT_FOOD

        elif self.state == State.WAITING_EXTRACT_FOOD:
            if self.hri_client.is_extract_done():
                extracted_text = self.hri_client.get_extracted_info()
                self.get_logger().info(f"📝 Extracto obtenido (platos principales): {extracted_text}")

                if extracted_text and extracted_text != "ERROR":
                    list_items = extracted_text.strip('\n').split(";")
                    n = len(list_items)
                    self.get_logger().info(f"✅ Se han extraído {n} ítems de interés.")
                    phrase = self.order_to_string(list_items, "Y de comer, has pedido: ")
                    self.hri_client.start_speaking(phrase)
                    self.state = State.WAITING_ECHO_FOOD
                else:
                    # Si falla, pasamos directo a procesar los postres
                    self.get_logger().info("🔍 Enviando texto al servicio Extract (postres)...")
                    self.hri_client.start_extract("postres", self.user_response)
                    self.state = State.WAITING_EXTRACT_DESSERT

        elif self.state == State.WAITING_ECHO_FOOD:
            if self.hri_client.is_speaking_done():
                # Pasamos a procesar postres
                self.get_logger().info("🔍 Enviando texto al servicio Extract (postres)...")
                self.hri_client.start_extract("postres", self.user_response)
                self.state = State.WAITING_EXTRACT_DESSERT

        elif self.state == State.WAITING_EXTRACT_DESSERT:
            if self.hri_client.is_extract_done():
                extracted_text = self.hri_client.get_extracted_info()
                self.get_logger().info(f"📝 Extracto obtenido (postres): {extracted_text}")

                if extracted_text and extracted_text != "ERROR":
                    list_items = extracted_text.strip('\n').split(";")
                    n = len(list_items)
                    self.get_logger().info(f"✅ Se han extraído {n} ítems de interés.")
                    phrase = self.order_to_string(list_items, "De postre, quieres: ")
                    self.hri_client.start_speaking(phrase)
                    self.state = State.WAITING_ECHO_DESSERT
                else:
                    self.state = State.DONE

        elif self.state == State.WAITING_ECHO_DESSERT:
            if self.hri_client.is_speaking_done():
                self.state = State.DONE

        elif self.state == State.DONE:
            self.get_logger().info("🎉 Demostración finalizada.")
            self.timer.cancel()
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = HRIExample2Client()
    
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
