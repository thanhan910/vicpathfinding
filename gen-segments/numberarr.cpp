#include "numberarr.h"

#include <cstring>
#include <cmath>

#include <string>
#include <iostream>
#include <vector>
#include <map>

Number::Number(char* abs_int_value, int int_size, char* dec_value, int dec_size, bool is_negative) :
    is_negative(is_negative),
    integer_value(abs_int_value),
    integer_size(int_size),
    decimal_value(dec_value),
    decimal_size(dec_size) {}

Number::Number(char* abs_int_value, int int_size, bool is_negative) :
    Number(abs_int_value, int_size, nullptr, 0, is_negative) {}

Number::Number() : Number(nullptr, 0, nullptr, 0, false) {}


// Copy constructor
Number::Number(const Number& other) :
    integer_size(other.integer_size), decimal_size(other.decimal_size), is_negative(other.is_negative) {
    integer_value = new char[integer_size];
    std::memcpy(integer_value, other.integer_value, integer_size);
    if (decimal_size > 0) {
        decimal_value = new char[decimal_size];
        std::memcpy(decimal_value, other.decimal_value, decimal_size);
    } else {
        decimal_value = nullptr;
    }
}

// Move constructor
Number::Number(Number&& other) noexcept :
    integer_size(other.integer_size), decimal_size(other.decimal_size), is_negative(other.is_negative),
    integer_value(other.integer_value), decimal_value(other.decimal_value) {
    other.integer_value = nullptr;
    other.decimal_value = nullptr;
}

// Copy assignment operator
Number& Number::operator=(const Number& other) {
    if (this != &other) {
        integer_size = other.integer_size;
        decimal_size = other.decimal_size;
        is_negative = other.is_negative;

        if (integer_value != nullptr) {
            delete[] integer_value;
            integer_value = nullptr;
        }

        if (decimal_value != nullptr) {
            delete[] decimal_value;
            decimal_value = nullptr;
        }

        integer_value = new char[integer_size];
        std::memcpy(integer_value, other.integer_value, integer_size);
        if (decimal_size > 0) {
            decimal_value = new char[decimal_size];
            std::memcpy(decimal_value, other.decimal_value, decimal_size);
        } else {
            decimal_value = nullptr;
        }
    }
    return *this;
}

// Move assignment operator
Number& Number::operator=(Number&& other) noexcept {
    if (this != &other) {
        integer_size = other.integer_size;
        decimal_size = other.decimal_size;
        is_negative = other.is_negative;

        if (integer_value != nullptr) {
            delete[] integer_value;
            integer_value = nullptr;
        }

        if (decimal_value != nullptr) {
            delete[] decimal_value;
            decimal_value = nullptr;
        }

        integer_value = other.integer_value;
        decimal_value = other.decimal_value;
        other.integer_value = nullptr;
        other.decimal_value = nullptr;
    }
    return *this;
}

// Destructor
Number::~Number() {
    if (integer_value != nullptr) {
        delete[] integer_value;
        integer_value = nullptr;
    }
    if (decimal_value != nullptr) {
        delete[] decimal_value;
        decimal_value = nullptr;
    }
}


Number::Number(const int &num)
{
    is_negative = num < 0;
    integer_size = 0;
    decimal_size = 0;
    if (num == 0)
    {
        integer_value = new char[1];
        integer_value[0] = '0';
        return;
    }
    integer_size = 0;
    int n = num;
    while (n != 0)
    {
        n /= 10;
        integer_size++;
    }
    integer_value = new char[integer_size];
    n = num;
    for (int i = 0; i < integer_size; i++)
    {
        integer_value[integer_size - i - 1] = n % 10 + '0';
        n /= 10;
    }
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

    integer_size = str_size - is_negative;
    decimal_size = 0;
    
    for (int i = is_negative; i < str_size; i++)
    {
        char c = str[i];
        if (c < '0' || c > '9')
        {
            if (c == '.')
            {
                if (decimal_size > 0)
                {
                    throw std::invalid_argument("Invalid number format (multiple decimal points)");
                }
                else
                {
                    integer_size = i - is_negative;
                    decimal_size = str_size - integer_size - is_negative - 1;
                }
            }
            else
            {
                throw std::invalid_argument("Invalid number format (invalid character)");
            }
        }
    }
    integer_value = new char[integer_size];
    if (decimal_size > 0)
    {
        decimal_value = new char[decimal_size];
    }
    else
    {
        decimal_value = nullptr;
    }
    for (int i = is_negative; i < str_size; i++)
    {
        char c = str[i];
        if (i < integer_size + is_negative)
        {
            integer_value[i - is_negative] = c;
        }
        else if (decimal_size > 0)
        {
            decimal_value[i - integer_size - is_negative - 1] = c;
        }
    }

    integer_value[integer_size] = '\0'; // Null terminate the integer part
    if (decimal_size > 0) {
        decimal_value[decimal_size] = '\0'; // Null terminate the decimal part
    }
}

Number::Number(const std::string &str) : Number(str.c_str()) {}

std::string Number::to_string() const
{
    std::string result = "";
    if (is_negative)
    {
        result += "-";
    }
    for (int i = 0; i < integer_size; i++)
    {
        result += integer_value[i];
    }
    if (decimal_size > 0)
    {
        result += ".";
        for (int i = 0; i < decimal_size; i++)
        {
            result += decimal_value[i];
        }
    }
    return result;
}

double Number::to_double() const
{
    double result = 0;
    for (int i = 0; i < integer_size; i++)
    {
        result = result * 10 + integer_value[i] - '0';
    }
    if (decimal_size > 0)
    {
        double dec_result = 0;
        for (int i = 0; i < decimal_size; i++)
        {
            dec_result = dec_result * 10 + decimal_value[i] - '0';
        }
        result += dec_result / pow(10, decimal_size);
    }
    if (is_negative)
    {
        result = -result;
    }
    return result;
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
    if (is_negative != other.is_negative)
    {
        return false;
    }

    if (integer_size != other.integer_size)
    {
        return false;
    }

    for (int i = 0; i < integer_size; i++)
    {
        if (integer_value[i] != other.integer_value[i])
        {
            return false;
        }
    }

    if (decimal_size != other.decimal_size)
    {
        return false;
    }

    for (int i = 0; i < decimal_size; i++)
    {
        if (decimal_value[i] != other.decimal_value[i])
        {
            return false;
        }
    }

    return true;
}

bool Number::operator>(const Number &other) const
{
    if (is_negative && !other.is_negative)
    {
        return false;
    }
    else if (!is_negative && other.is_negative)
    {
        return true;
    }
    else if (is_negative && other.is_negative)
    {
        return -(*this) < -other;
    }

    if (integer_size != other.integer_size)
    {
        return integer_size > other.integer_size;
    }

    for (int i = 0; i < integer_size; i++)
    {
        if (integer_value[i] != other.integer_value[i])
        {
            return integer_value[i] > other.integer_value[i];
        }
    }

    for (int i = 0; i < decimal_size; i++)
    {
        if (i >= other.decimal_size)
        {
            return true;
        }

        if (decimal_value[i] > other.decimal_value[i])
        {
            return true;
        }
        else if (decimal_value[i] < other.decimal_value[i])
        {
            return false;
        }
    }

    return false;
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
Number Number::operator+(const Number &other) const
{
    if (is_negative && !other.is_negative)
    {
        return other - -(*this);
    }
    else if (!is_negative && other.is_negative)
    {
        return *this - -other;
    }
    else if (is_negative && other.is_negative)
    {
        return -(-(*this) + -other);
    }

    if (integer_size > other.integer_size)
    {
        return other + *this;
    }
    short int remembers = 0;
    short int dsum;

    int new_dec_size = std::max(decimal_size, other.decimal_size);
    char *new_dec_value;
    if (new_dec_size > 0)
    {
        new_dec_value = new char[new_dec_size];
        for (int i = new_dec_size - 1; i >= 0; i--)
        {
            short int d1, d2;
            if (i < decimal_size)
            {
                d1 = decimal_value[i] - '0';
            }
            else
            {
                d1 = 0;
            }
            if (i < other.decimal_size)
            {
                d2 = other.decimal_value[i] - '0';
            }
            else
            {
                d2 = 0;
            }
            dsum = d1 + d2 + remembers;
            new_dec_value[i] = dsum % 10 + '0';
            remembers = dsum / 10;
        }
    }
    else
    {
        new_dec_value = nullptr;
    }

    int new_int_size = std::max(integer_size, other.integer_size) + 1;
    char *new_reverse_int_value = new char[new_int_size];
    for (int i = 0; i < new_int_size; i++)
    {
        int i1 = integer_size - i - 1;
        int i2 = other.integer_size - i - 1;
        short int d1, d2;
        if (i1 < 0)
        {
            d1 = 0;
        }
        else
        {
            d1 = integer_value[i1] - '0';
        }
        if (i2 < 0)
        {
            d2 = 0;
        }
        else
        {
            d2 = other.integer_value[i2] - '0';
        }
        dsum = d1 + d2 + remembers;
        new_reverse_int_value[new_int_size - i - 1] = dsum % 10 + '0';
        remembers = dsum / 10;
    }
    char *new_int_value;
    for (int i = 0; i < new_int_size; i++)
    {
        if (new_reverse_int_value[i] != '0')
        {
            new_int_value = new char[new_int_size - i];
            for (int j = i; j < new_int_size; j++)
            {
                new_int_value[j - i] = new_reverse_int_value[j];
            }
            break;
        }
    }

    return Number(new_int_value, new_int_size, new_dec_value, new_dec_size);
}

// Minus operator
Number Number::operator-(const Number &other) const
{
    if (is_negative && !other.is_negative)
    {
        return -(-(*this) + other);
    }
    else if (!is_negative && other.is_negative)
    {
        return *this + -other;
    }
    else if (is_negative && other.is_negative)
    {
        return -(-(*this) - -other);
    }

    if (*this < other)
    {
        return -(other - *this);
    }

    short int remembers = 0;
    short int ddiff;

    int new_dec_size = std::max(decimal_size, other.decimal_size);
    char *new_dec_value;
    if (new_dec_size > 0)
    {
        new_dec_value = new char[new_dec_size];
        for (int i = new_dec_size - 1; i >= 0; i--)
        {
            short int d1, d2;
            if (i < decimal_size)
            {
                d1 = decimal_value[i] - '0';
            }
            else
            {
                d1 = 0;
            }
            if (i < other.decimal_size)
            {
                d2 = other.decimal_value[i] - '0';
            }
            else
            {
                d2 = 0;
            }
            ddiff = d1 - d2 - remembers;
            if (ddiff < 0)
            {
                ddiff += 10;
                remembers = 1;
            }
            else
            {
                remembers = 0;
            }
            new_dec_value[i] = ddiff + '0';
        }
    }
    else
    {
        new_dec_value = nullptr;
    }

    int new_int_size = integer_size;
    char *new_reverse_int_value = new char[new_int_size];

    for (int i = 0; i < new_int_size; i++)
    {
        int i1 = integer_size - i - 1;
        int i2 = other.integer_size - i - 1;
        short int d1, d2;
        if (i1 < 0)
        {
            d1 = 0;
        }
        else
        {
            d1 = integer_value[i1] - '0';
        }
        if (i2 < 0)
        {
            d2 = 0;
        }
        else
        {
            d2 = other.integer_value[i2] - '0';
        }
        ddiff = d1 - d2 - remembers;
        if (ddiff < 0)
        {
            ddiff += 10;
            remembers = 1;
        }
        else
        {
            remembers = 0;
        }
        new_reverse_int_value[new_int_size - i - 1] = ddiff + '0';
    }

    char *new_int_value;
    for (int i = 0; i < new_int_size; i++)
    {
        if (new_reverse_int_value[i] != '0')
        {
            new_int_value = new char[new_int_size - i];
            for (int j = i; j < new_int_size; j++)
            {
                new_int_value[j - i] = new_reverse_int_value[j];
            }
            break;
        }
    }

    return Number(new_int_value, new_int_size, new_dec_value, new_dec_size);
}

std::ostream &operator<<(std::ostream &os, const Number &num)
{
    if (num.is_negative)
    {
        os << "-";
    }

    os << num.integer_value;

    if (num.decimal_size > 0)
    {
        os << ".";
        os << num.decimal_value;
    }
    return os;
}
