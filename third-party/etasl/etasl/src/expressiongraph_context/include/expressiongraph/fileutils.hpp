#ifndef EXPRESSIONGRAPH_FILEUTILS_HPP
#define EXPRESSIONGRAPH_FILEUTILS_HPP

#include <string>

std::string getEnvVar(std::string const& key);

bool checkFilename(const std::string& fn);

std::string append_path(const std::string& dirname, const std::string& filename);

std::string getFullFileName(const std::string& _path, const std::string& filename);

#endif

