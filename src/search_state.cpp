#include "search_state.hpp"

#include <cassert>
#include <unordered_set>

#include "common_defs.hpp"
#include "debug.hpp"


#ifdef DEBUGSS
  #define DBGSS(STMT) DBG(STMT)
#else
  #define DBGSS(STMT) 0
#endif


unsigned SearchState::fingers_num_ = 0;


SearchState::SearchState() : f_(0), g_(0), h_(0), id_(-1), force_closure_(-1),
                             quality_(0), parent_(nullptr)
{
  assert(fingers_num_ >= MIN_VALID_FINGERS_NUM);

  fingers_super_contacts_.resize(fingers_num_);

  //** TODO: Finding a way to resize super contact sets before using
}


SearchState::SearchState(unsigned _id) : f_(0), g_(0), h_(0), force_closure_(-1),
                                         quality_(0), parent_(nullptr)
{
  assert(fingers_num_ >= MIN_VALID_FINGERS_NUM);

  fingers_super_contacts_.resize(fingers_num_);

  id_ = _id;

  //** TODO: Finding a way to resize super contact sets before using
}


SearchState::~SearchState()
{
  parent_.reset();
  parent_ = nullptr;
}


SearchState::SearchState(SearchState&& _in)
{
  *this = std::move(_in);
}


SearchState& SearchState::operator =(SearchState&& _rhs)
{
  if (this != &_rhs)
  {
    f_ = std::move(_rhs.f_);
    g_ = std::move(_rhs.g_);
    h_ = std::move(_rhs.h_);
    id_ = std::move(_rhs.id_);
    force_closure_ = std::move(_rhs.force_closure_);
    quality_ = std::move(_rhs.quality_);
    parent_ = std::move(_rhs.parent_);
    fingers_super_contacts_ = std::move(_rhs.fingers_super_contacts_);

    _rhs.parent_.reset();
    _rhs.parent_ = nullptr;
  }
}


void SearchState::setFingersNum(const unsigned& _f_num)
{
  assert(_f_num >= MIN_VALID_FINGERS_NUM);

  fingers_num_ = _f_num;
}


void SearchState::setEndEffectorType(const END_EFFECTOR_T& _end_eff)
{
  if (_end_eff == parallel_jaw) //0
  {
    setFingersNum(2);
  }
}


void SearchState::print() const
{
  DBGSS("========== State ID ==========> " << id_ << "\n"
        << "fingers num: " << fingers_num_ << "\n"
        << "quality val: " << quality_ << "\n"
        << "force closure: " << force_closure_ << "\n"
        << "f: " << f_ << "\n"
        << "g: " << g_ << "\n"
        << "h: " << h_ << "\n"
        << "parent: " << parent_ << "\n"
        << "Super Contacts:\n");

  for (size_t i = 0; i < fingers_num_; ++i)
  {
    DBGSS("<" << i << "> ==>\n" << fingers_super_contacts_[i] << std::endl);
  }
}


bool SearchState::isValid() const
{
  return !(singleElementSuperContacts() && similarSuperContacts());
}


std::ostream& operator << (std::ostream& _stream, const SearchState& _ss)
{
  _stream << "========== State ID ==========> " << _ss.id_ << "\n"
          << "fingers num: " << _ss.fingers_num_ << "\n"
          << "quality val: " << _ss.quality_ << "\n"
          << "force closure: " << _ss.force_closure_ << "\n"
          << "f: " << _ss.f_ << "\n"
          << "g: " << _ss.g_ << "\n"
          << "h: " << _ss.h_ << "\n"
          << "parent: " << _ss.parent_ << "\n"
          << "Super Contacts:\n";

  for (size_t i = 0; i < _ss.fingers_num_; ++i)
  {
    _stream << "<" << i << "> ==>\n" << _ss.fingers_super_contacts_[i] << std::endl;
  }

  return _stream;
}


bool SearchState::similarSuperContacts() const
{
  SuperContact firstSC = fingers_super_contacts_[0];

  for (size_t i =  1; i < fingers_num_; ++i)
  {
    if (firstSC.cPsSet_ != fingers_super_contacts_[i].cPsSet_)
      return false;
  }

  return true;
}


bool SearchState::singleElementSuperContacts() const
{
  for (const auto& curr_super_contact : fingers_super_contacts_)
  {
    if (curr_super_contact.cPsSet_.size() > 1)
    {
      return false;
    }
  }

  return true;
}


unsigned SearchState::numCPsWORepeat() const
{
  unsigned numCPsUnion = 0;

  std::unordered_set<unsigned> repeat_avoid;

  for (const auto& curr_super_contact : fingers_super_contacts_)
  {
    for (const auto& curr_contact : curr_super_contact.cPsSet_)
    {
      if (repeat_avoid.find(curr_contact) != repeat_avoid.end())
      {
        continue;
      }

      repeat_avoid.insert(curr_contact);
      numCPsUnion++;
    }
  }

  return numCPsUnion;
}


void SearchState::updateForceClosureStatus()
{
  // Positive force closure is force closure, otherwise not
  // Acoording to the literature
  if (quality_ > 0)
  {
    force_closure_ = 1;
  }

  force_closure_ = 0;
}


bool SearchState::operator ==(const SearchState& _rhs) const
{
  bool res = false;

  unsigned similarSCsNum = 0;

  std::unordered_set<unsigned> repeat_avoid;

  for (const auto& curr_sc_lhs : fingers_super_contacts_)
  {
    res = false;

    for (size_t i = 0; i < fingers_num_; ++i)
    {
      if ((repeat_avoid.find(i) == repeat_avoid.end()) &&
          (curr_sc_lhs.cPsSet_.size() == (_rhs.fingers_super_contacts_[i]).cPsSet_.size()) &&
          (curr_sc_lhs.cPsSet_ == (_rhs.fingers_super_contacts_[i]).cPsSet_))
      {
        similarSCsNum++;

        repeat_avoid.insert(i);

        res = true;

        break; // In the case if there is more than one similar super contact set for our current supper contact in rhs
      }
    }

    if (!res) // With one super contact non similar, we can return false
    {
      return false;
    }
  }

  return true;
}
