#ifndef __SUPER_CONTACT_HPP_
#define __SUPER_CONTACT_HPP_


#include <vector>
#include <iostream>


typedef std::vector<unsigned> ContactPointsSet;
typedef std::vector<ContactPointsSet> ContactPointSet_Set;


class SuperContact
{
  public:
    SuperContact() = default;
    virtual ~SuperContact() = default;

    SuperContact(const SuperContact&) = default;
    SuperContact& operator =(const SuperContact&) = default;

    SuperContact(SuperContact&& _in);
    SuperContact& operator =(SuperContact&& _rhs);

    ContactPointsSet cPsSet_;

    inline size_t getSize() const { return cPsSet_.size(); }

    void print() const;

    friend std::ostream& operator <<(std::ostream& _stream, const SuperContact& _sc);
};


typedef std::vector<SuperContact> SuperContactSet;
typedef std::vector<SuperContactSet> SuperContactSet_Set;


#endif // __SUPER_CONTACT_HPP_
