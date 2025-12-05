#include "JsonHelper.hpp"
#include <fstream>
#include <iostream>
#include <filesystem>

using nlohmann::json_schema::json_validator;
namespace fs = std::filesystem;

nlohmann::json load_json_from_file(const fs::path& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filepath.string());
    }
    
    nlohmann::json j;
    try {
        file >> j;
    } catch (const nlohmann::json::parse_error& e) {
        throw std::runtime_error("JSON parse error in file '" + filepath.string() + "': " + e.what());
    }
    
    return j;
}

bool validate_with_detailed_errors(const nlohmann::json& input_json, const fs::path& schema_filepath) {
    try {
        // Load schema
        nlohmann::json schema = load_json_from_file(schema_filepath);
        
        // Create validator with error handler
        json_validator validator(schema);
        
        // Set up custom error handler for detailed reporting
        validator.set_root_schema(schema);
        
        try {
            validator.validate(input_json);
            std::cout << "JSON validation successful based on the schema" << std::endl;
            return true;
        } catch (const std::exception& e) {
            std::cout << "JSON validation failed with error:" << std::endl;
            std::cout << "  " << e.what() << std::endl;
            return false;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error loading schema: " << e.what() << std::endl;
        return false;
    }
}