#pragma once

#include <source_location>
#include <stdexcept>
#include <string>

namespace collision_restraint
{

[[maybe_unused]] static std::string source_prefix(
  std::source_location location = std::source_location::current())
{
  return std::string(location.file_name()) + ":" + std::to_string(location.line()) + ":" +
         std::to_string(location.column()) + ":" + location.function_name() + ": ";
}

}  // namespace collision_restraint
