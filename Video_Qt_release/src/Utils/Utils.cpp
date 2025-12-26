
#include <iostream>
#include <vector>
#include <filesystem> 



namespace fs = std::filesystem;


std::vector<std::string> getImagePaths(const std::string& directory, const std::string& extension = ".png") {
    std::vector<std::string> paths;
    if (!fs::exists(directory) || !fs::is_directory(directory)) {
        std::cerr << "Error: Directory does not exist or is not a directory: " << directory << std::endl;
        return paths;
    }
    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.is_regular_file() && entry.path().extension() == extension) {
            paths.push_back(entry.path().string());
        }
    }
    std::sort(paths.begin(), paths.end());
    return paths;
}
