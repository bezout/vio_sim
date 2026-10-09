#pragma once
#include "long_motion.hpp"
#include <map>
#include <istream>
namespace long_motion {
// Flat YAML-compatible subset: scalar keys and bracketed numeric arrays only.
// This is NOT a general YAML parser. Reuse the host project's real parser when
// it exists, and pass its resolved scalar/array values through set_option().
std::map<std::string,std::string> read_flat_options(std::istream&);
void set_option(Config&,const std::string& key,const std::string& value);
} // namespace long_motion
