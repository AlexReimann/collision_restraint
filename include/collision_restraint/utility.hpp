#pragma once

#include <format>
#include <source_location>
#include <string>

namespace collision_restraint
{

[[maybe_unused]] static std::string source_prefix(
  std::source_location location = std::source_location::current())
{
  return std::format(
    "{}:{}:{}:{}", location.file_name(), location.line(), location.column(),
    location.function_name());
}

}  // namespace collision_restraint
