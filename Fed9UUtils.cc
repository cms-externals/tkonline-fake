#include <vector>
#include <string>
#include <exception>
#include <stdexcept>
#include <stdint.h>

typedef uint8_t u8;

#define THROW_RUNTIME                                        \
  throw std::runtime_error(                                  \
      "Unsupported call to trackerDAQ library, function: " + \
      std::string(__PRETTY_FUNCTION__))

namespace Fed9U {
class Fed9UAddress {
 public:
  enum { UNSPECIFIED = 0xFF, FEBROADCAST = 14, BACKEND = 9, VME = 10 };
  Fed9UAddress();
  Fed9UAddress(u8 fedChannel, u8 channelApv = UNSPECIFIED,
               u8 apvStrip = UNSPECIFIED);
  Fed9UAddress& setFedApv(u8 fedApv);
};

Fed9UAddress::Fed9UAddress() { THROW_RUNTIME; };
Fed9UAddress::Fed9UAddress(u8 fedChannel, u8 channelApv, u8 apvStrip) {
  THROW_RUNTIME;
};
Fed9UAddress& Fed9UAddress::setFedApv(u8 fedApv) { THROW_RUNTIME; };

class Fed9UStripDescription {};

class Fed9UStrips {
 public:
  std::vector<Fed9UStripDescription> getApvStrips(Fed9UAddress fedApv) const;
  Fed9UStripDescription& getStrip(Fed9UAddress fedStrip);
  const Fed9UStripDescription& getStrip(Fed9UAddress fedStrip) const;
  void setStrip(Fed9UAddress fedStrip, const Fed9UStripDescription& value);
};

std::vector<Fed9UStripDescription> Fed9UStrips::getApvStrips(
    Fed9UAddress fedApv) const {
  THROW_RUNTIME;
};
Fed9UStripDescription& Fed9UStrips::getStrip(Fed9UAddress fedStrip) {
  THROW_RUNTIME;
};
const Fed9UStripDescription& Fed9UStrips::getStrip(
    Fed9UAddress fedStrip) const {
  THROW_RUNTIME;
};
void Fed9UStrips::setStrip(Fed9UAddress fedStrip,
                           const Fed9UStripDescription& value) {
  THROW_RUNTIME;
};
}