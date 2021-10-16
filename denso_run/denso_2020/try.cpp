#include <iostream>
#include <string>
#include <cstdlib>
#include <vector>
#include <filesystem>

// #include <iostream>
// #include <string>
// #include <vector>
    
std::vector<std::string> split(std::string str, char del) {
    int first = 0;
    int last = str.find_first_of(del);
    
    std::vector<std::string> result;
    
    while (first < str.size()) {
        std::string subStr(str, first, last - first);
    
        result.push_back(subStr);
    
        first = last + 1;
        last = str.find_first_of(del, first);
    
        if (last == std::string::npos) {
            last = str.size();
        }
    }
    
    return result;
}
    
int main() {
    // std::string str = "samurai,engineer,programmer,se";
    std::string path = "/home/tsuchida/attou/rer/imga/fdf.jpg";
    char del = '/';
    const char *tmp = std::getenv("HOME");
    std::string env_var(tmp ? tmp : "");
    if (env_var.empty()) {
        std::cerr << "[ERROR] No such variable found!" << std::endl;
        exit(EXIT_FAILURE);
    }
    std::string ss;
    ss = env_var + "/";

    std::cout << env_var << "/";
    for (int i = 0; i < split(path, del).size() - 2; i++) {
        if (i == split(path, del).size() - 4) {
            ss = ss + split(path, del)[i+3];
            std::cout << split(path, del)[i+3] << std::endl;

            break;
        }
        ss = ss + split(path, del)[i + 3] + "/";
        std::filesystem::create_directory(ss);
        std::cout << split(path, del)[i + 3] << "/";
    }
    
    return 0;
}

// const char *ENV_VAR = "HOME";

// int main() {
//     const char *tmp = std::getenv("HOME");
//     std::string env_var(tmp ? tmp : "");
//     if (env_var.empty()) {
//         std::cerr << "[ERROR] No such variable found!" << std::endl;
//         exit(EXIT_FAILURE);
//     }

//     std::cout << "HOME : " << env_var << std::endl;

//     exit(EXIT_SUCCESS);

// }
