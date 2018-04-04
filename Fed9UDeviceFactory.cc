#include <vector>
#include <exception>
#include <stdexcept>
#include <string>
#include <stdint.h>
/*
#ifdef CMS_TK_64BITS

#include <stdint.h>

  typedef int8_t  i8;  ///< Signed 8-bit integer.
  typedef int16_t i16; ///< Signed 16-bit integer.
  typedef int32_t  i32; ///< Signed 32-bit integer.

  typedef uint8_t  u8;  ///< Unsigned 8-bit integer.
  typedef uint16_t u16; ///< Unsigned 16-bit integer.
  typedef uint32_t  u32; ///< Unsigned 32-bit integer.

#else

  typedef signed char  i8;  ///< Signed 8-bit integer.
  typedef signed short i16; ///< Signed 16-bit integer.
  typedef signed long  i32; ///< Signed 32-bit integer.

  typedef unsigned char  u8;  ///< Unsigned 8-bit integer.
  typedef unsigned short u16; ///< Unsigned 16-bit integer.
  typedef unsigned long  u32; ///< Unsigned 32-bit integer.


#endif
*/

typedef int16_t i16;
typedef uint16_t u16;

#define THROW_RUNTIME                                        \
  throw std::runtime_error(                                  \
      "Unsupported call to trackerDAQ library, function: " + \
      std::string(__PRETTY_FUNCTION__))

namespace Fed9U {
class Fed9UDescription {};

//  class Fed9UDeviceFactory : public DeviceFactoryInterface {
class Fed9UDeviceFactory {
  static void vectorCopy(std::vector<Fed9UDescription*>& dst,
                         std::vector<Fed9UDescription*>& src);
  std::vector<Fed9UDescription*>* getFed9UDescriptions(std::string partition,
                                                       i16 major = -1,
                                                       i16 minor = -1,
                                                       int maskMajor = 0,
                                                       int maskMinor = 0);
  Fed9UDeviceFactory& setFed9UDescriptions(std::vector<Fed9UDescription*> f,
                                           std::string partition,
                                           u16* versionMajor = NULL,
                                           u16* versionMinor = NULL,
                                           int majVersion = 0);
  Fed9UDeviceFactory& setUsingStrips(bool usingStrips);
};
void Fed9UDeviceFactory::vectorCopy(std::vector<Fed9UDescription*>& dst,
                                    std::vector<Fed9UDescription*>& src) {
  THROW_RUNTIME;
};
std::vector<Fed9UDescription*>* Fed9UDeviceFactory::getFed9UDescriptions(
    std::string partition, i16 major, i16 minor, int maskMajor, int maskMinor) {
  THROW_RUNTIME;
};
Fed9UDeviceFactory& Fed9UDeviceFactory::setFed9UDescriptions(
    std::vector<Fed9UDescription*> f, std::string partition, u16* versionMajor,
    u16* versionMinor, int majVersion) {
  THROW_RUNTIME;
};
Fed9UDeviceFactory& Fed9UDeviceFactory::setUsingStrips(bool usingStrips) {
  THROW_RUNTIME;
};
}