#include "super_contact.hpp"

#include "debug.hpp"


#ifdef DEBUGSC
  #define DBGSC(STMT) DBG(STMT)
#else
  #define DBGSC(STMT) 0
#endif


SuperContact::SuperContact(SuperContact&& _in)
{
  cPsSet_ = std::move(_in.cPsSet_);
}


SuperContact& SuperContact::operator =(SuperContact&& _rhs)
{
  if (this != &_rhs)
  {
    cPsSet_ = std::move(_rhs.cPsSet_);
  }
}


void SuperContact::print() const
{
  DBGSC("----- Super Contact Info -----\n");
  DBGSC("Size: " << getSize() << std::endl);

  for (const auto& curr_contact : cPsSet_)
  {
    DBGSC(curr_contact << "  ");
  }

  DBGSC(std::endl);
}


std::ostream& operator << (std::ostream& _stream, const SuperContact& _sc)
{
  _stream << "----- Super Contact Info -----\n";
  _stream << "Size: " << _sc.getSize() << std::endl;

  for (const auto& curr_contact : _sc.cPsSet_)
  {
    _stream << curr_contact << "  ";
  }

  return _stream;
}
