#include "SlaveComm.h"

SlaveComm *globalSlave = nullptr;   // Puntero global para acceder a la instancia del objeto desde el callback


// CALLBACK GLOBAL
void ESPNOWReceiveCallback(uint8_t *mac, uint8_t *incomingMsg, uint8_t len) {
  if (globalSlave == nullptr) return;   // Si no hay instancia, no procesa nada

  char buffer[100];                     // Buffer para almacenar el mensaje recibido
  memcpy(buffer, incomingMsg, len);     // Copia el mensaje recibido al buffer
  buffer[len] = 0;                      // Añade terminación de cadena

  globalSlave->processIncoming(buffer); // Llama al método de la instancia para procesar el mensaje
}


// CONSTRUCTOR
SlaveComm::SlaveComm() {
  _id = 0;                    // ID inicial del esclavo
  _angle = 0;                 // Ángulo inicial
  _distance = 0;              // Distancia inicial
  _out = 0;                   // Estado/salida inicial
  _masterMACSet = false;      // Aún no se ha configurado la MAC del maestro
}


// INICIALIZACIÓN
bool SlaveComm::begin(const char *ssid, const char *password) {

  globalSlave = this;         // Guarda esta instancia en el puntero global

  WiFi.mode(WIFI_STA);        // Pone el módulo en modo estación
  WiFi.begin(ssid, password); // Conecta al WiFi
  while (WiFi.status() != WL_CONNECTED) delay(100);  // Espera hasta conectarse

  // Obtiene la MAC del propio dispositivo y la guarda en _myMACAddress
  sscanf(WiFi.macAddress().c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
         &_myMACAddress[0], &_myMACAddress[1], &_myMACAddress[2],
         &_myMACAddress[3], &_myMACAddress[4], &_myMACAddress[5]);

  if (esp_now_init() != 0) return false;  // Inicializa ESP-NOW y verifica errores

  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);           // Permite enviar y recibir
  esp_now_register_recv_cb(ESPNOWReceiveCallback);     // Registra el callback

  if (_masterMACSet)                                   // Si ya sabemos la MAC del maestro...
    esp_now_add_peer(_masterMACAddress, ESP_NOW_ROLE_CONTROLLER, 1, NULL, 0);

  return true;                                         // Inicialización exitosa
}


// GUARDAR MAC DEL MAESTRO
void SlaveComm::setMasterMACAddress(uint8_t mac[6]) {
  memcpy(_masterMACAddress, mac, 6);   // Copia la MAC proporcionada al atributo
  _masterMACSet = true;                // Marca que ya está configurada
}


// PROCESAR MENSAJE RECIBIDO
void SlaveComm::processIncoming(const char *msg) {
  int ID;
  float ang, dist;
  int out;

  // Parsea el mensaje con formato esperado
  if (sscanf(msg, "id=%d, ang=%f, dist=%f, Out=%d",
             &ID, &ang, &dist, &out) == 4) {

    if (ID == _id) {   // Solo procesa si el mensaje es para este esclavo
      _angle = ang;    // Guarda ángulo
      _distance = dist;// Guarda distancia
      _out = out;      // Guarda salida
      sendOK();        // Envía confirmación al maestro
    }
  }
}


// ENVIAR OK AL MAESTRO
void SlaveComm::sendOK() {
  char msgOut[20];                   // Mensaje de salida
  sprintf(msgOut, "OK id=%d", _id);  // Construye mensaje OK con el ID

  esp_now_send(_masterMACAddress, (uint8_t *)msgOut, strlen(msgOut)); // Envía al maestro
}
