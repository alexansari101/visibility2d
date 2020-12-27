#pragma once

#include <fstream>
#include <iterator>
#include <type_traits>
#include <vector>

namespace util {

template <typename V>
void csv(const V &v, const char *outputFilename, bool append = false) {
  auto mode = append ? std::ofstream::app : std::ofstream::out;
  std::ofstream ofs(outputFilename, mode);
  for (size_t i = 0; i < v.size(); i++) {
    if (i == 0) {
      ofs << v[i];
    } else {
      ofs << "," << v[i];
    }
  }
  ofs << std::endl;
  ofs.close();
};

} // namespace util
