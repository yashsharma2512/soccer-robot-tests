#pragma once
#include "Arduino.h"
#include <map>
#include <vector>
#include <cstring>
struct FakeNVS {
  std::map<std::string,std::vector<uint8_t>> blobs;
  bool failBegin=false,failPut=false,failRemove=false;
  unsigned writes=0,removes=0;
};
extern FakeNVS fakeNvs;
class Preferences {
  bool opened=false;
 public:
  bool begin(const char *,bool=false) { opened=!fakeNvs.failBegin; return opened; }
  bool isKey(const char *key) { return opened && fakeNvs.blobs.count(key)!=0; }
  size_t getBytesLength(const char *key) { return isKey(key) ? fakeNvs.blobs[key].size() : 0; }
  size_t getBytes(const char *key,void *data,size_t length) {
    if (!isKey(key) || fakeNvs.blobs[key].size()>length) return 0;
    size_t n=fakeNvs.blobs[key].size(); memcpy(data,fakeNvs.blobs[key].data(),n); return n;
  }
  size_t putBytes(const char *key,const void *data,size_t length) {
    fakeUs+=80000; // A flash write stalls more than a gyro sample-gap limit.
    if (!opened || fakeNvs.failPut) return 0;
    const uint8_t *bytes=static_cast<const uint8_t *>(data);
    fakeNvs.blobs[key]=std::vector<uint8_t>(bytes,bytes+length); ++fakeNvs.writes;
    return length;
  }
  bool remove(const char *key) {
    fakeUs+=60000;
    if (!opened || fakeNvs.failRemove) return false;
    ++fakeNvs.removes; return fakeNvs.blobs.erase(key)!=0;
  }
};
