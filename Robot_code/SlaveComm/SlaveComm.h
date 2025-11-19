#ifndef SLAVECOMM_h
#define SLAVECOMM_h

#include <Arduino.h>              // Biblioteca base de Arduino
#include <ESP8266WiFi.h>          // Manejo de WiFi en ESP8266
#include <espnow.h>               // Biblioteca para comunicación ESP-NOW

class SlaveComm {
  private:
    int _id;                      // ID del dispositivo esclavo
    float _angle;                 // Valor recibido: ángulo
    float _distance;              // Valor recibido: distancia
    int _out;                     // Valor recibido: salida o estado

    uint8_t _masterMACAddress[6]; // Dirección MAC del maestro
    uint8_t _myMACAddress[6];     // Dirección MAC propia del esclavo
    bool _masterMACSet;           // Indicador de si la MAC del maestro ya fue configurada

  public:
    SlaveComm();                  // Constructor de la clase

    bool begin(const char *ssid, const char *password); // Inicializa WiFi y ESP-NOW

    int getID()        { return _id; }        // Obtener ID del esclavo
    float getAngle()   { return _angle; }     // Obtener ángulo recibido
    float getDistance(){ return _distance; }  // Obtener distancia recibida
    int getOut()       { return _out; }       // Obtener valor de salida

    void setID(int id) { _id = id; }          // Establecer ID del esclavo
    void setMasterMACAddress(uint8_t mac[6]); // Configurar dirección MAC del maestro

    void sendOK();                            // Enviar mensaje de confirmación al maestro

    void processIncoming(const char *msg);    // Procesar mensaje recibido
};

void ESPNOWReceiveCallback(uint8_t *mac, uint8_t *incomingMsg, uint8_t len); // Callback de recepción de ESP-NOW

extern SlaveComm *globalSlave;               // Puntero global a la instancia del esclavo

#endif
