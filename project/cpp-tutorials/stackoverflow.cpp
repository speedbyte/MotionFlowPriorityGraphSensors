


const int CONST_VALUE1 = 10; // Solution
//extern const int CONST_VALUE1; // Error

template< int N >
struct A
{
};

template<char c>
struct t
{
    static const char value = c;
};

template <unsigned N>
constexpr char test1(const char (&arr)[N], unsigned i)
{
    return arr[i];
}

template <unsigned N>
constexpr char test2(const char (&arr)[N], unsigned i)
{
    return t<arr[i]>::value;
}

int main() {
    A<CONST_VALUE1> A1; // the value of ‘CONST_VALUE1’ is not usable in a constant expression
    char a = test1("Test", 0); //Compiles OK
    char b = test2("Test", 0); //error: non-type template argument
    //is not a constant expression
    return 0;
}
