#include "helper.hpp"

namespace passform{
namespace util{

// splits a std::string into vector<string> at a delimiter
std::vector<std::string> split(const std::string &s, const std::string &delim)
{
  std::vector<std::string> splitted;
  auto start = 0U;
  auto end = s.find(delim);
  while (end != std::string::npos)
  {
    splitted.push_back( s.substr(start, end - start) );
    start = end + delim.length();
    end = s.find(delim, start);
  }
  splitted.push_back( s.substr(start, end));
  return splitted;
}

} // ns util
} // ns passform
