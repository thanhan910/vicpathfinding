#ifndef NUMBER_H
#define NUMBER_H

#include <string>
#include <iostream>

class Number
{
private:
    bool is_negative;
    char *integer_value;
    int integer_size;
    char *decimal_value;
    int decimal_size;

public:
    
    Number(char* abs_int_value, int int_size, char* dec_value, int dec_size, bool is_negative = false);
    Number(char* abs_int_value, int int_size, bool is_negative = false);
    Number();
    Number(const Number& other); // Copy constructor
    Number(Number&& other) noexcept; // Move constructor
    Number& operator=(const Number& other); // Copy assignment operator
    Number& operator=(Number&& other) noexcept; // Move assignment operator
    ~Number();
    
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