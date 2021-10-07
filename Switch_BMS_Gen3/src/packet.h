#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

#include <defines.h>
#include <pack_state.h>

#define PACK_ID_SIZE 21

using PacketJson = StaticJsonDocument<480>;

class Packet
{
public:
    char* packId();

    void setAvrSignature(byte *signature);

    PacketJson &identityPacket();
    PacketJson &dataPacket(PackState &state);

private:
    byte *_signature;
    char _packId[PACK_ID_SIZE];
    PacketJson _doc;

    void _clearPacket();
};