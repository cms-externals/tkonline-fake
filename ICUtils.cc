#include <stdexcept>
#include <string>

#define THROW_RUNTIME                                        \
  throw std::runtime_error(                                  \
      "Unsupported call to trackerDAQ library, function: " + \
      std::string(__PRETTY_FUNCTION__))

namespace ICUtils {

class ICException : public std::exception {
 public:
  ICException(const std::string& desc);
};

ICException::ICException(const std::string& desc) { THROW_RUNTIME; };
}