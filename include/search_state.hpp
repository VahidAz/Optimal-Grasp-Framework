#ifndef __SEARCH_STATE_HPP_
#define __SEARCH_STATE_HPP_


#include <memory>

#include "super_contact.hpp"
#include "common_defs.hpp"


class SearchState;
typedef std::shared_ptr<SearchState> SearchStateSPtr;


class SearchState
{
  public:
    SearchState();
    SearchState(unsigned _id);
    virtual ~SearchState();

    SearchState(const SearchState&) = default;
    SearchState& operator =(const SearchState&) = default;

    SearchState(SearchState&& _in);
    SearchState& operator =(SearchState&& _rhs);

    float f_,
          g_,
          h_,
          quality_;

    unsigned id_;

    int force_closure_;

    SearchStateSPtr parent_;

    SuperContactSet fingers_super_contacts_;

    static unsigned fingers_num_;

    static void setFingersNum(const unsigned& _f_num);
    static void setEndEffectorType(const END_EFFECTOR_T& _end_eff);

    bool isValid() const;

    unsigned numCPsWORepeat() const;

    void updateForceClosureStatus();

    bool singleElementSuperContacts() const;

    void print() const;

    bool operator ==(const SearchState& _rhs) const;

    friend std::ostream& operator <<(std::ostream& _stream, const SearchState& _ss);

  private:
    bool similarSuperContacts() const;
};


typedef std::vector<SearchStateSPtr> SearchStateSPtrSet;

typedef std::shared_ptr<const SearchState> SearchStateCSPtr;


#endif // __SEARCH_STATE_HPP_
