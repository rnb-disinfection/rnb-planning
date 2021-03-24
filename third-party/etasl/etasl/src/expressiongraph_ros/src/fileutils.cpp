#include <expressiongraph/fileutils.hpp>
#include <stdlib.h>
#include <fstream>

std::string getEnvVar(std::string const& key)
{
    char const* val = getenv(key.c_str()); 
    return val == NULL ? std::string() : std::string(val);
}


bool checkFilename(const std::string& fn) {
    using namespace std;
    ifstream is(fn.c_str());
    return !is.fail();
}

std::string append_path(const std::string& dirname, const std::string& filename) {
         if (dirname.size()!=0) {
            return dirname + "/" + filename;
        } else {
            return filename;
        }
}

std::string getFullFileName(const std::string& _path, const std::string& filename) {
    using namespace std;
    std::string path = _path;
    std::string fullname;
    size_t i = path.find(":");
    while (i != string::npos) {
        fullname = append_path( path.substr(0,i), filename);
        if (checkFilename(fullname)) {
            return fullname;
        } 
        path = path.substr(i+1);
        i = path.find(":");
    }
    fullname = append_path( path, filename);
    if (checkFilename(fullname)) {
        return fullname;
    } 
    return "";
}


