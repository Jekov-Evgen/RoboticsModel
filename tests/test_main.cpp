#include "test_harness.hpp"

#include <iostream>

int main() {
    int failures = 0;
    for (const auto& test : test::registry()) {
        try {
            test.body();
            std::cout << "PASS " << test.name << '\n';
        } catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL " << test.name << ": " << error.what() << '\n';
        }
    }
    std::cout << (failures ? "FAILED: " : "All tests passed; failures: ") << failures << " of "
              << test::registry().size() << '\n';
    return failures ? 1 : 0;
}
