// Copyright Daniel Wallin 2008. Use, modification and distribution is
// subject to the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)

#ifndef LUABIND_TYPEID_081227_HPP
# define LUABIND_TYPEID_081227_HPP

# include <boost/operators.hpp>
# include <typeinfo>
# include <luabind/detail/primitives.hpp>
# include <string>
#ifdef __GNUC__
    #include <cxxabi.h>
    #include <malloc.h>
    #include <boost/regex.hpp>
#endif

namespace luabind {

#ifdef __GNUC__
    inline std::string demangle( const std::string& name) {
        int status;
        char* buf=abi::__cxa_demangle(name.c_str(), 0,0, &status);
        std::string fullname; 
        boost::regex re("(\\w+\\:\\:)");
        switch(status) {
            case 0:
                fullname=buf;
                break;
            case -1:
                fullname="MEMORY ALLOC FAILED";
                break;
            case -2:
                fullname="INVALID NAME";
                break;
            case -3:
                fullname="INVALID NAME";
                break;
        } 
        free(buf);
        return boost::regex_replace(fullname,re,"");
    }
#else
    inline std::string demangle( const std::string& name) {
            return name;
    }
#endif


# ifdef BOOST_MSVC
#  pragma warning(push)
// std::type_info::before() returns int, rather than bool.
// At least on MSVC7.1, this is true for the comparison
// operators as well.
#  pragma warning(disable:4800)
# endif

class type_id
  : public boost::less_than_comparable<type_id>
{
public:
    type_id()
      : id(&typeid(detail::null_type))
    {}

    type_id(std::type_info const& id)
      : id(&id)
    {}

    bool operator!=(type_id const& other) const
    {
        return *id != *other.id;
    }

    bool operator==(type_id const& other) const
    {
        return *id == *other.id;
    }

    bool operator<(type_id const& other) const
    {
        return id->before(*other.id);
    }
    char const* name() const
    {
        return id->name();
    }

private:
    std::type_info const* id;
};

# ifdef BOOST_MSVC
#  pragma warning(pop)
# endif

} // namespace luabind

#endif // LUABIND_TYPEID_081227_HPP

