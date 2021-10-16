#include <iostream>
#include <string>
#include <cstdlib>



const char *ENV_VAR = "HOME";

int main() {
    const char *tmp = std::getenv("HOME");
    std::string env_var(tmp ? tmp : "");
    if (env_var.empty()) {
        std::cerr << "[ERROR] No such variable found!" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << "HOME : " << env_var << std::endl;

    exit(EXIT_SUCCESS);

}
