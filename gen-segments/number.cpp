#include "number.h"

#include <cstring>
#include <cmath>

#include <string>
#include <iostream>
#include <vector>
#include <map>


bool is_greater(const std::string &a, const std::string &b)
{
    bool a_is_negative = a[0] == '-';
    bool b_is_negative = b[0] == '-';
    size_t a_decimal = a.find('.');
    size_t b_decimal = b.find('.');
    if (a_is_negative && !b_is_negative)
    {
        return false;
    }
    else if (!a_is_negative && b_is_negative)
    {
        return true;
    }
    else if (a_is_negative && b_is_negative)
    {
        return !is_greater(a.substr(1), b.substr(1));
    }
    std::string a_integer_part = a_decimal == std::string::npos ? a : a.substr(0, a_decimal);
    std::string a_decimal_part = a_decimal == std::string::npos ? "" : a.substr(a_decimal + 1);
    std::string b_integer_part = b_decimal == std::string::npos ? b : b.substr(0, b_decimal);
    std::string b_decimal_part = b_decimal == std::string::npos ? "" : b.substr(b_decimal + 1);

    if (a_integer_part.size() != b_integer_part.size())
    {
        return a_integer_part.size() > b_integer_part.size();
    }

    else if (a_integer_part != b_integer_part)
    {
        return a_integer_part > b_integer_part;
    }

    else {
        return a_decimal_part > b_decimal_part;
    }

    return false;
}



Number::Number(std::string integer_value, std::string decimal_value, bool is_negative) :
    integer_part(integer_value), decimal_part(decimal_value), is_negative(is_negative) {}

Number::Number() : Number("", "") {}

// // Copy constructor
// Number::Number(const Number& other) :
// integer_part(other.integer_part), decimal_part(other.decimal_part), is_negative(other.is_negative) { }

// // Move constructor
// Number::Number(Number&& other) noexcept :
// integer_part(other.integer_part), decimal_part(other.decimal_part), is_negative(other.is_negative) {
//     other.integer_part = "";
//     other.decimal_part = "";
// }

// // Copy assignment operator
// Number& Number::operator=(const Number& other) {
//     if (this != &other) {
//         integer_part = other.integer_part;
//         decimal_part = other.decimal_part;
//         is_negative = other.is_negative;
//     }
//     return *this;
// }

// // Move assignment operator
// Number& Number::operator=(Number&& other) noexcept {
//     if (this != &other) {
//         integer_part = other.integer_part;
//         decimal_part = other.decimal_part;
//         is_negative = other.is_negative;
//         other.integer_part = "";
//         other.decimal_part = "";
//     }
//     return *this;
// }

Number::Number(const int &num)
{
    is_negative = num < 0;
    integer_part = std::to_string(abs(num));
    decimal_part = "";
    is_negative = false;
}

Number::Number(const char* str) : Number()
{
    bool str_empty = (str[0] == '\0');
    size_t str_size = strlen(str);
    if (str_empty)
    {
        throw std::invalid_argument("Invalid number format (empty string)");
    }

    is_negative = str[0] == '-';

    if (is_negative)
    {
        if (str_size == 1)
        {
            throw std::invalid_argument("Invalid number format (negative sign only)");
        }
    }
    bool found_decimal = false;
    for (int i = is_negative; i < str_size; i++)
    {
        char c = str[i];
        if (c < '0' || c > '9')
        {
            if (c == '.')
            {
                if (found_decimal)
                {
                    throw std::invalid_argument("Invalid number format (multiple decimal points)");
                }
                else
                {
                    found_decimal = true;
                }
            }
            else
            {
                throw std::invalid_argument("Invalid number format (invalid character)");
            }
        }
        else {
            if (found_decimal)
            {
                decimal_part += c;
            }
            else
            {
                integer_part += c;
            }
        }
    }
    // Remove leading zeros
    size_t i = 0;
    while (i < integer_part.size() && integer_part[i] == '0')
    {
        i++;
    }
    integer_part = integer_part.substr(i);
    i = decimal_part.size();
    while (i > 0 && decimal_part[i - 1] == '0')
    {
        i--;
    }
    decimal_part = decimal_part.substr(0, i);
}

Number::Number(const std::string &str) : Number(str.c_str()) {}

std::string Number::to_string() const
{
    std::string result = "";
    if (is_negative)
    {
        result += "-";
    }
    result += integer_part;
    if (decimal_part.size() > 0)
    {
        result += ".";
        result += decimal_part;
    }
    return result;
}

double Number::to_double() const
{
    double result = 0;
    for (int i = 0; i < integer_part.size(); i++)
    {
        result += (integer_part[i] - '0') * pow(10, integer_part.size() - i - 1);
    }
    for (int i = 0; i < decimal_part.size(); i++)
    {
        result += (decimal_part[i] - '0') * pow(10, -(i + 1));
    }
    return is_negative ? -result : result;
}

// Reverse operator
Number Number::operator-() const
{
    Number result = *this;
    result.is_negative = !is_negative;
    return result;
}

bool Number::operator==(const Number &other) const
{
    return integer_part == other.integer_part && decimal_part == other.decimal_part && is_negative == other.is_negative;
}

bool Number::operator>(const Number &other) const {
    return is_negative != other.is_negative ? is_negative : is_greater(to_string(), other.to_string());
}

bool Number::operator<(const Number &other) const
{
    return !(*this == other || *this > other);
}

bool Number::operator>=(const Number &other) const
{
    return !(*this < other);
}

bool Number::operator<=(const Number &other) const
{
    return !(*this > other);
}

bool Number::operator!=(const Number &other) const
{
    return !(*this == other);
}

// Plus operator
Number Number::operator+(const Number &other) const {
    if(is_negative != other.is_negative) {
        // Different signs - convert to subtraction
        Number pos_other = other;
        pos_other.is_negative = false;
        return is_negative ? other - *this : *this - pos_other;
    }

    // Pad decimal parts to same length
    std::string dec1 = decimal_part;
    std::string dec2 = other.decimal_part;
    size_t max_dec = std::max(dec1.length(), dec2.length());
    dec1.resize(max_dec, '0');
    dec2.resize(max_dec, '0');

    // Add decimal parts
    std::string result_dec;
    int carry = 0;
    for(int i = max_dec - 1; i >= 0; i--) {
        int sum = (dec1[i] - '0') + (dec2[i] - '0') + carry;
        carry = sum / 10;
        result_dec = char(sum % 10 + '0') + result_dec;
    }

    // Add integer parts
    std::string result_int;
    size_t i = integer_part.length() - 1;
    size_t j = other.integer_part.length() - 1;
    while(i != size_t(-1) || j != size_t(-1) || carry) {
        int sum = carry;
        if(i != size_t(-1)) sum += integer_part[i--] - '0';
        if(j != size_t(-1)) sum += other.integer_part[j--] - '0';
        carry = sum / 10;
        result_int = char(sum % 10 + '0') + result_int;
    }

    // Construct result
    std::string result = result_int;
    if(!result_dec.empty()) result += "." + result_dec;
    if(is_negative) result = "-" + result;
    return Number(result);
}


// Minus operator
Number Number::operator-(const Number &other) const {
    if(is_negative != other.is_negative) {
        // Different signs - convert to addition
        Number pos_other = other;
        pos_other.is_negative = false;
        return is_negative ? -pos_other + *this : *this + pos_other;
    }
    else if (is_negative && other.is_negative) {
        // Both negative - convert to subtraction
        return -other - -(*this);
    }
    else if (*this < other) {
        // Negative result - convert to subtraction
        return -(other - *this);
    }

    // Pad decimal parts to same length
    std::string dec1 = decimal_part;
    std::string dec2 = other.decimal_part;
    size_t max_dec = std::max(dec1.length(), dec2.length());
    dec1.resize(max_dec, '0');
    dec2.resize(max_dec, '0');

    // Subtract decimal parts
    std::string result_dec;
    int borrow = 0;
    for(int i = max_dec - 1; i >= 0; i--) {
        int diff = (dec1[i] - '0') - (dec2[i] - '0') - borrow;
        if(diff < 0) {
            diff += 10;
            borrow = 1;
        } else {
            borrow = 0;
        }
        result_dec = char(diff + '0') + result_dec;
    }

    // Subtract integer parts
    std::string result_int;
    size_t i = integer_part.length() - 1;
    size_t j = other.integer_part.length() - 1;
    while(i != size_t(-1) || j != size_t(-1) || borrow) {
        int diff = -borrow;
        if(i != size_t(-1)) diff += integer_part[i--] - '0';
        if(j != size_t(-1)) diff -= other.integer_part[j--] - '0';
        if(diff < 0) {
            diff += 10;
            borrow = 1;
        } else {
            borrow = 0;
        }
        result_int = char(diff + '0') + result_int;
    }

    // Construct result
    std::string result = result_int;
    if(!result_dec.empty()) result += "." + result_dec;
    if(is_negative) result = "-" + result;
    return Number(result);
}

std::ostream &operator<<(std::ostream &os, const Number &num)
{
    if (num.is_negative)
    {
        os << "-";
    }
    os << num.integer_part;
    if (num.decimal_part.size() > 0)
    {
        os << ".";
        os << num.decimal_part;
    }
    return os;
}
