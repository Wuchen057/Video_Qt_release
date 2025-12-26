#pragma once
#include <iostream>
#include <vector>
#include <filesystem> 

namespace fs = std::filesystem;


std::vector<std::string> getImagePaths(const std::string& directory, const std::string& extension = ".png");