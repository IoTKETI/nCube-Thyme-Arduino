
#ifndef _OTA_STORAGE_H_INCLUDED
#define _OTA_STORAGE_H_INCLUDED

class OTAStorage {
public:
  virtual int open() = 0;
  virtual size_t write(uint8_t) = 0;
  virtual void close() = 0;
  virtual void clear() = 0;
  virtual void apply() = 0;

  virtual long maxSize() {
    return ((256 * 1024) - 0x2000);
  }
};

#endif
