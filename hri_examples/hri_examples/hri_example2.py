import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from simple_hri_interfaces.srv import Speech
from simple_hri_interfaces.srv import Extract
from enum import Enum, auto

import time


class State(Enum):
    INIT = auto()
    WAITING_INTRO = auto()
    WAITING_INTRO_DELAY = auto()
    WAITING_USER_RESPONSE = auto()
    WAITING_EXTRACT_DRINK = auto()
    WAITING_ECHO_DRINK_DELAY = auto()
    WAITING_ECHO_DRINK = auto()
    WAITING_EXTRACT_FOOD = auto()
    WAITING_ECHO_FOOD_DELAY = auto()
    WAITING_ECHO_FOOD = auto()
    WAITING_EXTRACT_DESSERT = auto()
    WAITING_ECHO_DESSERT_DELAY = auto()
    WAITING_ECHO_DESSERT = auto()
    DONE = auto()


class HRIExample2(Node):

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
        self.extract_client = self.create_client(Extract, '/extract_service')
        while not self.extract_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/extract_service no disponible, esperando...')

        self.get_logger().info("✅ Clientes Extract, STT y TTS listos para usar.")
        
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
        if self.is_sleeping():
            return

        if self.state == State.INIT:
            self.get_logger().info("🤖 Iniciando demostración de HRI con Extract...")

            tts_req = Speech.Request()
            tts_req.text = "Hola. Vamos a probar la extracción de información. Imagina que soy un camarero y tú eres un cliente que va a hacer un pedido. ¿Qué te gustaría pedir de beber y de comer?"
            self.current_future = self.tts_client.call_async(tts_req)
            self.state = State.WAITING_INTRO

        elif self.state == State.WAITING_INTRO:
            if self.current_future and self.current_future.done():
                tts_response = self.current_future.result()
                if tts_response.success:
                    self.get_logger().info("✅ TTS ejecutado correctamente")
                else:
                    self.get_logger().error(f"❌ Error en TTS: {tts_response.debug}")

                # Darle tiempo a la locución y unos segundos extra antes de iniciar STT
                self.set_sleep(11.0) # 8.0 base + 3.0 de delay original de tu código
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

                self.get_logger().info("🔍 Enviando texto al servicio Extract (bebida)...")
                ext_req = Extract.Request()
                ext_req.text = self.user_response
                ext_req.interest = "bebida"
                self.current_future = None
                self.current_future = self.extract_client.call_async(ext_req)
                self.state = State.WAITING_EXTRACT_DRINK

        # ------------- BEBIDA -------------
        elif self.state == State.WAITING_EXTRACT_DRINK:
            if self.current_future and self.current_future.done():
                extract_response = self.current_future.result()
                self.get_logger().info(f"📝 Extracto obtenido (bebida): {extract_response.result}")

                if extract_response.result and extract_response.result != "ERROR":
                    list_items = extract_response.result.strip('\n').split(";")
                    n = len(list_items)
                    self.get_logger().info(f"✅ Se han extraído {n} ítems de interés.")
                    self.phrase_to_speak = self.order_to_string(list_items, "De beber, has pedido: ")
                    
                    # Hacemos el delay para que no pise nada anterior o suene natural
                    self.set_sleep(2.0)
                    self.state = State.WAITING_ECHO_DRINK_DELAY
                else:
                    self.get_logger().error(f"❌ Error en Extract: {extract_response.message}")
                    # Si falla pasamos a platos principales igual
                    self.get_logger().info("🔍 Enviando texto al servicio Extract (platos principales)...")
                    ext_req = Extract.Request()
                    ext_req.text = self.user_response
                    ext_req.interest = "platos principales"
                    self.current_future = None
                    self.current_future = self.extract_client.call_async(ext_req)
                    self.state = State.WAITING_EXTRACT_FOOD

        elif self.state == State.WAITING_ECHO_DRINK_DELAY:
            tts_req = Speech.Request()
            tts_req.text = self.phrase_to_speak
            self.current_future = None
            self.current_future = self.tts_client.call_async(tts_req)
            self.state = State.WAITING_ECHO_DRINK

        elif self.state == State.WAITING_ECHO_DRINK:
            if self.current_future and self.current_future.done():
                tts_response = self.current_future.result()
                if tts_response.success:
                    self.get_logger().info("✅ TTS ejecutado correctamente")
                else:
                    self.get_logger().error(f"❌ Error en TTS: {tts_response.debug}")

                # Pasamos al siguiente
                self.get_logger().info("🔍 Enviando texto al servicio Extract (platos principales)...")
                ext_req = Extract.Request()
                ext_req.text = self.user_response
                ext_req.interest = "platos principales"
                self.current_future = None
                self.current_future = self.extract_client.call_async(ext_req)
                self.state = State.WAITING_EXTRACT_FOOD

        # ------------- COMIDA -------------
        elif self.state == State.WAITING_EXTRACT_FOOD:
            if self.current_future and self.current_future.done():
                extract_response = self.current_future.result()
                self.get_logger().info(f"📝 Extracto obtenido (platos principales): {extract_response.result}")

                if extract_response.result and extract_response.result != "ERROR":
                    list_items = extract_response.result.strip('\n').split(";")
                    n = len(list_items)
                    self.get_logger().info(f"✅ Se han extraído {n} ítems de interés.")
                    self.phrase_to_speak = self.order_to_string(list_items, "Y de comer, has pedido: ")
                    
                    self.set_sleep(2.0)
                    self.state = State.WAITING_ECHO_FOOD_DELAY
                else:
                    self.get_logger().error(f"❌ Error en Extract: {extract_response.message}")
                    self.get_logger().info("🔍 Enviando texto al servicio Extract (postres)...")
                    ext_req = Extract.Request()
                    ext_req.text = self.user_response
                    ext_req.interest = "postres"
                    self.current_future = None
                    self.current_future = self.extract_client.call_async(ext_req)
                    self.state = State.WAITING_EXTRACT_DESSERT

        elif self.state == State.WAITING_ECHO_FOOD_DELAY:
            tts_req = Speech.Request()
            tts_req.text = self.phrase_to_speak
            self.current_future = None
            self.current_future = self.tts_client.call_async(tts_req)
            self.state = State.WAITING_ECHO_FOOD

        elif self.state == State.WAITING_ECHO_FOOD:
            if self.current_future and self.current_future.done():
                tts_response = self.current_future.result()
                if tts_response.success:
                    self.get_logger().info("✅ TTS ejecutado correctamente")
                else:
                    self.get_logger().error(f"❌ Error en TTS: {tts_response.debug}")

                # Pasamos al último
                self.get_logger().info("🔍 Enviando texto al servicio Extract (postres)...")
                ext_req = Extract.Request()
                ext_req.text = self.user_response
                ext_req.interest = "postres"
                self.current_future = None
                self.current_future = self.extract_client.call_async(ext_req)
                self.state = State.WAITING_EXTRACT_DESSERT

        # ------------- POSTRE -------------
        elif self.state == State.WAITING_EXTRACT_DESSERT:
            if self.current_future and self.current_future.done():
                extract_response = self.current_future.result()
                self.get_logger().info(f"📝 Extracto obtenido (postres): {extract_response.result}")

                if extract_response.result and extract_response.result != "ERROR":
                    list_items = extract_response.result.strip('\n').split(";")
                    n = len(list_items)
                    self.get_logger().info(f"✅ Se han extraído {n} ítems de interés.")
                    self.phrase_to_speak = self.order_to_string(list_items, "De postre, quieres: ")
                    
                    self.set_sleep(2.0)
                    self.state = State.WAITING_ECHO_DESSERT_DELAY
                else:
                    self.get_logger().error(f"❌ Error en Extract: {extract_response.message}")
                    self.state = State.DONE

        elif self.state == State.WAITING_ECHO_DESSERT_DELAY:
            tts_req = Speech.Request()
            tts_req.text = self.phrase_to_speak
            self.current_future = None
            self.current_future = self.tts_client.call_async(tts_req)
            self.state = State.WAITING_ECHO_DESSERT

        elif self.state == State.WAITING_ECHO_DESSERT:
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
    node = HRIExample2()
    
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
