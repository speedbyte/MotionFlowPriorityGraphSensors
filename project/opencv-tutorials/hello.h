//
// Created by veikas on 22.07.17.
//


#ifndef OPENCV_TUTORIAL_HELLO_H
#define OPENCV_TUTORIAL_HELLO_H

typedef void (Error::*fun_ptr)(); // fun_ptr takes void parameter and returns void. fun_ptr = any_function();

class Error {
public:

private:
    std::stringstream m_ss;
    bool m_valid;
    const char* m_prefix;

public:
    Error(const char* prefix);

    Error& operator<< <> (const fun_ptr fun);

    std::string toString();

    void invalidate();

    void stub();

    void runtime_exception();

    static fun_ptr condition(const bool& cond);

    static fun_ptr path_exists(const boost::filesystem::path& path);

    static fun_ptr is_file(const boost::filesystem::path& path);

    static fun_ptr is_directory(const boost::filesystem::path& path);
};

#define ssprefix "\n\tin file: " __FILE__ ":" BOOST_PP_STRINGIZE(__LINE__) "\n\tmessage:\n\t\t"
#define sserr do { vtd_framework::utils::Error( ssprefix )
#define sscond( cond ) vtd_framework::utils::Error::condition((cond))
#define ssthrow &vtd_framework::utils::Error::runtime_exception; } while(false)
#define sspexits( path ) vtd_framework::utils::Error::path_exists( path ) << "no such file or directory: " << path
#define sspfile( path ) vtd_framework::utils::Error::is_file( path ) << "file not found: " << path
#define sspdir( path ) vtd_framework::utils::Error::is_directory( path ) << "directory not found: " << path
#define ssequal( v0 , v1 ) sscond( (v0) != (v1) ) << "expected: " << (v1) << " got: " << (v0)


#endif //OPENCV_TUTORIAL_HELLO_H
