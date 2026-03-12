# hri_client

Cliente reutilizable para servicios de simple_hri (STT, TTS, Extract, YesNo).

## Descripción

Este paquete proporciona una clase `HRIClient` que encapsula la funcionalidad de interacción humano-robot (HRI) proporcionada por `simple_hri`, facilitando su uso en aplicaciones y behavior trees.

## Características

- **Speech-to-Text (STT)**: Transcribir audio a texto
- **Text-to-Speech (TTS)**: Convertir texto a audio y reproducirlo
- **Extract**: Extraer información específica del audio (ej. nombres, colores, números)
- **YesNo**: Detectar respuestas afirmativas/negativas del audio
- API asíncrona con métodos `start_*`, `is_*_done()` y `get_*_result()`
- Suscripción al topic `/listened_text` para recibir actualizaciones de STT

## Uso

```cpp
#include "hri_client/hri_client.hpp"

auto hri_client = std::make_shared<HRIClient>();

// Esperar a que los servicios estén disponibles
if (!hri_client->wait_for_services()) {
  // Servicios no disponibles
  return;
}

// Uso asíncrono - Speech-to-Text
hri_client->start_listen();
while (!hri_client->is_listen_done()) {
  rclcpp::spin_some(hri_client);
  // Hacer otras cosas mientras escucha...
}
auto text = hri_client->get_listened_text();

// Uso asíncrono - Text-to-Speech
hri_client->start_speaking("Hola, ¿cómo estás?");
while (!hri_client->is_speaking_done()) {
  rclcpp::spin_some(hri_client);
  // Hacer otras cosas mientras habla...
}

// Uso asíncrono - Extract
hri_client->start_extract("name", "");  // Vacío = usar audio
while (!hri_client->is_extract_done()) {
  rclcpp::spin_some(hri_client);
}
auto name = hri_client->get_extracted_info();

// Uso asíncrono - YesNo
hri_client->start_yesno("");  // Vacío = usar audio
while (!hri_client->is_yesno_done()) {
  rclcpp::spin_some(hri_client);
}
auto answer = hri_client->get_yesno_result();  // "yes" o "no"
```

## Dependencias

- rclcpp
- std_srvs
- std_msgs
- simple_hri_interfaces

