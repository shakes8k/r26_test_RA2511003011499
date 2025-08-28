#define _USE_MATH_DEFINES
#include "ublox_reader.h"
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

static void ubxChecksum(const uint8_t *data, size_t len, uint8_t &ck_a, uint8_t &ck_b) {
  ck_a = 0; ck_b = 0;
  for (size_t i = 0; i < len; ++i) {
    ck_a = static_cast<uint8_t>(ck_a + data[i]);
    ck_b = static_cast<uint8_t>(ck_b + ck_a);
  }
}

static int NAV_POSLLH(const uint8_t *payload, classId *gps) {
  memcpy(&gps->iTOW,   payload + 0,  4);
  memcpy(&gps->lon,    payload + 4,  4);
  memcpy(&gps->lat,    payload + 8,  4);
  memcpy(&gps->height, payload + 12, 4);
  memcpy(&gps->hMSL,   payload + 16, 4);
  memcpy(&gps->hAcc,   payload + 20, 4);
  memcpy(&gps->vAcc,   payload + 24, 4);
  return 0;
}

static vector<uint8_t> hexToBytes(const string &rawHex) {
  vector<uint8_t> bytes;
  stringstream ss(rawHex);
  string token;
  while (ss >> token) {
    // allow tokens like "B5" or "0xB5"
    if (token.rfind("0x", 0) == 0 || token.rfind("0X", 0) == 0) token = token.substr(2);
    bytes.push_back(static_cast<uint8_t>(stoul(token, nullptr, 16)));
  }
  return bytes;
}

int decodeUBX(uint8_t *buffer, classId *gps) {
  // obviously had to create, Provided test cases did not have a full UBX frame (which were sync bytes)
  size_t off = 0;
  if (buffer[0] == 0xB5 && buffer[1] == 0x62) off = 2;

  uint8_t cls = buffer[off + 0];
  uint8_t id  = buffer[off + 1];
  uint16_t len = static_cast<uint16_t>(buffer[off + 2]) |
                 (static_cast<uint16_t>(buffer[off + 3]) << 8);

  // Only NAV-POSLLH (class=0x01, id=0x02, len=28) handled here.
  if (cls != 0x01 || id != 0x02 || len != 28) return 1;

  const uint8_t *payload = buffer + off + 4;

  // Verify checksum over class, id, lenL, lenH, payload
  uint8_t ck_a_expected = buffer[off + 4 + len];
  uint8_t ck_b_expected = buffer[off + 4 + len + 1];

  uint8_t ck_a = 0, ck_b = 0;
  ubxChecksum(buffer + off, 4 + len, ck_a, ck_b);
  if (ck_a != ck_a_expected || ck_b != ck_b_expected) return 1;

  return NAV_POSLLH(payload, gps);
}

GPS gpsFromData(const classId &gps) {
  GPS out;
  out.lat = gps.lat * 1e-7;         // degrees
  out.lon = gps.lon * 1e-7;         // degrees
  out.height = gps.height / 1000.0; // meters (ellipsoid height)
  return out;
}
//  - honestly this code was mostly written by gpt, i had low information on how to extract data into cpp
pair<GPS, GPS> readUbloxFile(const string &filename) {
  ifstream file(filename);
  if (!file.is_open()) {
    cerr << "Error: cannot open file " << filename << endl;
    return {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  }

  string rawStart, rawGoal;
  getline(file, rawStart);
  getline(file, rawGoal);
  file.close();

  vector<uint8_t> startBytes = hexToBytes(rawStart);
  vector<uint8_t> goalBytes  = hexToBytes(rawGoal);

  auto ensureChecksum = [](vector<uint8_t> &buf) {
    if (buf.empty()) return;

    size_t off = (buf.size() >= 2 && buf[0] == 0xB5 && buf[1] == 0x62) ? 2 : 0;

    if (buf.size() < off + 4) return; // not enough for class,id,len
    uint16_t len = static_cast<uint16_t>(buf[off + 2]) |
                   (static_cast<uint16_t>(buf[off + 3]) << 8);

    const size_t needNoCk = off + 4 + len;
    const size_t needWithCk = needNoCk + 2;
    if (buf.size() == needNoCk) {
      uint8_t ck_a = 0, ck_b = 0;
      ubxChecksum(buf.data() + off, 4 + len, ck_a, ck_b);
      buf.push_back(ck_a);
      buf.push_back(ck_b);
    }
  }; //WTRMK - RA2511003011499

  ensureChecksum(startBytes);
  ensureChecksum(goalBytes);

  classId dStart{}, dGoal{};

  auto decodeIfReady = [&](vector<uint8_t> &buf, classId *dst) {
    if (buf.empty()) return;
    size_t off = (buf.size() >= 2 && buf[0] == 0xB5 && buf[1] == 0x62) ? 2 : 0;
    if (buf.size() < off + 4) return;
    uint16_t len = static_cast<uint16_t>(buf[off + 2]) |
                   (static_cast<uint16_t>(buf[off + 3]) << 8);
    if (buf.size() >= off + 4 + len + 2) {
      decodeUBX(buf.data(), dst);
    }
  };

  decodeIfReady(startBytes, &dStart);
  decodeIfReady(goalBytes,  &dGoal);

  return {gpsFromData(dStart), gpsFromData(dGoal)};
}
