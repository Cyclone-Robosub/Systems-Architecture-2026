#pragma once

#include <string>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <nlohmann/json-schema.hpp>
namespace fs = std::filesystem;
nlohmann::json load_json_from_file(const fs::path& filepath);
bool validate_with_detailed_errors(const nlohmann::json& input_json, const fs::path& schema_filepath);
