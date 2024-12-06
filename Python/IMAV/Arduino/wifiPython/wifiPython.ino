#include <WiFiNINA.h>
#include <Servo.h>

Servo myServo;  // Crea un objeto servo para controlar un servomotor

int servoPin = 9;

// Configura los detalles de tu red WiFi
const char* ssid = "Linksys02037";
const char* password = "aqkm92ep7w";

// Inicializa el servidor en el puerto 80
WiFiServer server(80);

void setup() {
  // Inicia el monitor serie
  //Serial.begin(9600);
  myServo.attach(servoPin);  // Conecta el servomotor al pin especificado
  //while (!Serial) {
  //  ; // Espera a que se inicie el puerto serie
  //}

  // Intenta conectarse a la red WiFi
  while (WiFi.begin(ssid, password) != WL_CONNECTED) {
    //Serial.print("Conectando a ");
    //Serial.println(ssid);
    delay(10000);
  }

  // Una vez conectado, inicia el servidor
  server.begin();
  //Serial.print("Conectado a la red WiFi. IP: ");
  //Serial.println(WiFi.localIP());

  // Configura el pin del LED
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // Revisa si hay un cliente disponible
  WiFiClient client = server.available();
  if (client) {
    //Serial.println("Cliente conectado");
    while (client.connected()) {
      if (client.available()) {
        char command = client.read();
        if (command == '1') {
          digitalWrite(LED_BUILTIN, HIGH); // Enciende el LED
          for (int angle = 0; angle <= 180; angle++) {
            myServo.write(angle);    // Configura el ángulo del servomotor
            //Serial.print("Ángulo del servomotor: ");
            //Serial.println(angle);   // Imprime el ángulo actual
            delay(15);               // Espera 15ms para que el servomotor alcance el ángulo
          }
        } else if (command == '0') {
          digitalWrite(LED_BUILTIN, LOW); // Apaga el LED
          for (int angle = 180; angle >= 0; angle--) {
            myServo.write(angle);    // Configura el ángulo del servomotor
            //Serial.print("Ángulo del servomotor: ");
            //Serial.println(angle);   // Imprime el ángulo actual
            delay(15);               // Espera 15ms para que el servomotor alcance el ángulo
          }
        }
        client.print("Comando recibido: ");
        client.println(command);
      }
    }
    client.stop();
    //Serial.println("Cliente desconectado");
  }
}