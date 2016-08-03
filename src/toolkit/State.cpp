#include "State.hpp"

namespace rtql8 {
    namespace toolkit {
        std::ostream& operator << (std::ostream& sout, const AbstractState& s) {
            s.save(sout);
            return sout;
        }

        std::istream& operator >> (std::istream& sin, AbstractState& s) {
            s.load(sin);
            return sin;
        }
    } // namespace toolkit
} // namespace rtql8
