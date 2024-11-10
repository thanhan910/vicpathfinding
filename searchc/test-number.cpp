#include <string>
#include <iostream>
#include <vector>
#include <map>
#include "number.h"

int main()
{
    // create multiple test cases
    std::string a = "20.47100005289473";
    std::string b = "100.2570987349827";
    Number num_a(a);
    Number num_b(b);
    std::cout << "Is " << a << " greater than " << b << "? " << ((num_a > num_b) ? "Yes" : "No") << std::endl;
    std::cout << "Is " << a << " less than " << b << "? " << ((num_a < num_b) ? "Yes" : "No") << std::endl;
    // Reverse of a
    Number num_a_rev = -num_a;
    std::cout << "Reverse of " << a << " is " << num_a_rev << std::endl;
    std::cout << "Integer size: " << num_a_rev.integer_part.size() << std::endl;
    for (int i = 0; i < num_a_rev.integer_part.size(); i++)
    {
        std::cout << num_a_rev.integer_part[i];
    }
    std::cout << std::endl;
    std::cout << "Decimal size: " << num_a_rev.decimal_part.size() << std::endl;
    std::cout << num_a_rev.decimal_part << std::endl;

    // Plus operator
    Number num_c = num_a + num_b;
    std::cout << num_c.integer_part << std::endl;
    std::cout << num_c.decimal_part << std::endl;
    std::cout << num_c << std::endl;
    std::cout << num_a + num_b << std::endl;
    std::cout << "Sum of " << a << " and " << b << " is " << num_c << std::endl;

    // Minus operator
    Number num_d = num_a - num_b;
    std::cout << "Difference of " << a << " and " << b << " is " << num_d << std::endl;
    return 0;
}