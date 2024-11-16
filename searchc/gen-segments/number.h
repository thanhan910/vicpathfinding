#ifndef NUMBER_H
#define NUMBER_H

#include <string>
#include <iostream>

bool is_greater(const std::string &a, const std::string &b);

class Number
{

public:

bool is_negative;
    std::string integer_part;
    std::string decimal_part;
    
    Number(std::string integer_value, std::string decimal_value, bool is_negative = false);
    Number();
    // Number(const Number& other); // Copy constructor
    // Number(Number&& other) noexcept; // Move constructor
    // Number& operator=(const Number& other); // Copy assignment operator
    // Number& operator=(Number&& other) noexcept; // Move assignment operator
    // ~Number();

    Number(const int &num);
    Number(const std::string &str);
    Number(const char* str);

    std::string to_string() const;
    double to_double() const;

    Number operator-() const;
    bool operator==(const Number &other) const;
    bool operator>(const Number &other) const;
    bool operator<(const Number &other) const;
    bool operator>=(const Number &other) const;
    bool operator<=(const Number &other) const;
    bool operator!=(const Number &other) const;
    Number operator+(const Number &other) const;
    Number operator-(const Number &other) const;

    friend std::ostream &operator<<(std::ostream &os, const Number &num);
};

#endif // NUMBER_H