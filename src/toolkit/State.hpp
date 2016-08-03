#ifndef TOOLKIT_STATE_H
#define TOOLKIT_STATE_H

#include <vector>
#include <istream>
#include <ostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include "Moreeigen.h"

namespace rtql8 {
    namespace toolkit {
        enum StateType {
            STATE_TYPE_INT,
            STATE_TYPE_DOUBLE,
            STATE_TYPE_STRING,
            STATE_TYPE_VECTOR2D,
            STATE_TYPE_VECTOR3D,
            STATE_TYPE_VECTORXD,
            STATE_TYPE_CUSTOM,
            NUM_STATE_TYPE
        };

        struct AbstractState;
        struct State;
        template <class T>
        struct StateArray;

        struct AbstractState {
            virtual void save(std::ostream& sout) const = 0;
            virtual std::string toString() const {
                std::stringstream sout;
                save(sout);
                return sout.str();
            }
            virtual void saveFile(const char* const filename) {
                std::ofstream fout(filename);
                save(fout);
            }

            virtual bool load(std::istream& sin) = 0;
            virtual bool fromString(const std::string& str) {
                std::stringstream sin(str);
                bool result = load(sin);
                return result;
            }
            virtual bool loadFile(const char* const filename) {
                std::ifstream fin(filename);
                if (!fin.is_open()) return false;
                if (!load(fin)) return false;
                return true;
            }
            
        protected:
            static bool consumeTo(std::istream& sin, char c) {
                char x;
                while(true) {
                    sin >> x;
                    if (sin.fail()) return false;
                    if (x == c) return true;
                }
                return false;
            }
            static char consumeTo(std::istream& sin, char c, char c2) {
                char x;
                while(true) {
                    sin >> x;
                    if (sin.fail()) return 0;
                    if (x == c)  return c;
                    if (x == c2) return c2;
                }
                return 0;
            }
            static bool consumeString(std::istream& sin, std::string* pStr) {
                if (!consumeTo(sin, '\"')) return false;
                std::string ret("");
                while(true) {
                    char x;
                    sin.get(x);
                    if (sin.fail()) return false;
                    if (x == '\"') {
                        (*pStr) = ret;
                        return true;
                    }
                    ret += x;
                }
                return false;
            }
        };

        std::ostream& operator << (std::ostream& sout, const AbstractState& s);
        std::istream& operator >> (std::istream& sin, AbstractState& s);


    
        struct StateEntry {
            StateType type;
            std::string name;
            int offset;
            StateEntry(StateType _t, const std::string& _n, int _o)
                : type(_t), name(_n), offset(_o) {}
        };
        
        typedef std::vector<StateEntry>::iterator SE_ITER;
        typedef std::vector<StateEntry>::const_iterator SE_CONST_ITER;

        struct State : public AbstractState {
            State() {
                entries.clear();
            }

            size_t numEntries() const { return entries.size(); }
            StateEntry* entry(const std::string& name) {
                for (SE_ITER i = entries.begin(); i != entries.end(); i++) {
                    StateEntry& se = (*i);
                    if (se.name == name) {
                        return &se;
                    }
                }
                return NULL;
            }
    
            std::vector<StateEntry> entries;
    
            void registerMember(const std::string& name, int* v) {
                int offset = (char* const)v - (char* const)(this);
                entries.push_back(StateEntry(STATE_TYPE_INT, name, offset));
            }

            void registerMember(const std::string& name, double* v) {
                int offset = (char* const)v - (char* const)(this);
                entries.push_back(StateEntry(STATE_TYPE_DOUBLE, name, offset));
            }

            void registerMember(const std::string& name, std::string* v) {
                int offset = (char* const)v - (char* const)(this);
                entries.push_back(StateEntry(STATE_TYPE_STRING, name, offset));
            }

            void registerMember(const std::string& name, Eigen::Vector2d* v) {
                int offset = (char* const)v - (char* const)(this);
                entries.push_back(StateEntry(STATE_TYPE_VECTOR2D, name, offset));
            }

            void registerMember(const std::string& name, Eigen::Vector3d* v) {
                int offset = (char* const)v - (char* const)(this);
                entries.push_back(StateEntry(STATE_TYPE_VECTOR3D, name, offset));
            }

            void registerMember(const std::string& name, Eigen::VectorXd* v) {
                int offset = (char* const)v - (char* const)(this);
                entries.push_back(StateEntry(STATE_TYPE_VECTORXD, name, offset));
            }

            void registerMember(const std::string& name, AbstractState* v) {
                int offset = (char* const)v - (char* const)(this);
                entries.push_back(StateEntry(STATE_TYPE_CUSTOM, name, offset));
            }

            void* const ptr(int offset) const {
                return (void*) (((char* const)this)+offset);
            }

            virtual void save(std::ostream& sout) const {
                using rtql8::toolkit::moreeigen::IO;
                sout << "{";
                for (SE_CONST_ITER i = entries.begin(); i != entries.end(); i++) {
                    if (i != entries.begin()) {
                        sout << ", ";
                    }
                    const StateEntry& se = (*i);
                    sout << "\"" << se.name << "\" : ";
                    switch(se.type) {
                    case STATE_TYPE_INT:
                        sout << *(int*)ptr(se.offset); break;
                    case STATE_TYPE_DOUBLE:
                        sout << *(double*)ptr(se.offset); break;
                    case STATE_TYPE_STRING:
                        sout << "\"" << *(std::string*)ptr(se.offset) << "\""; break;
                    case STATE_TYPE_VECTOR2D:
                        sout << IO(*(Eigen::Vector2d*)ptr(se.offset)); break;
                    case STATE_TYPE_VECTOR3D:
                        sout << IO(*(Eigen::Vector3d*)ptr(se.offset)); break;
                    case STATE_TYPE_VECTORXD:
                        sout << IO(*(Eigen::VectorXd*)ptr(se.offset)); break;
                    case STATE_TYPE_CUSTOM:
                        ((AbstractState*)ptr(se.offset))->save(sout); break;
                    default:
                        // std::cerr << "Invalid type: " << se.type << std::endl;
                        break;
                    }
                }
                sout << "}";
            }

            virtual bool load(std::istream& sin) {
                using rtql8::toolkit::moreeigen::IO;

                if (!consumeTo(sin, '{')) {
                	std::cout << "CONSUME BEGIN ERROR: " << (*this) << std::endl;
                	return false;
                }

                while(true) {
                    std::string name;
                    if (!consumeString(sin, &name)) {
                    	std::cout << "CONSUME NAME ERROR: " << (*this) << std::endl;
                    	return false;
                    }
                    const StateEntry* const pSE = entry(name);
                    if (pSE == NULL) {
                    	std::cout << "CONSUME pSE NULL ERROR: " << (*this) << std::endl;
                    	return false;
                    }
                    const StateEntry& se = *pSE;

                    if (!consumeTo(sin, ':')) {
                    	std::cout << "CONSUME COLUMN ERROR: " << (*this) << std::endl;
                    	return false;
                    }

                    switch(se.type) {
                    case STATE_TYPE_INT:
                        sin >> *(int*)ptr(se.offset);
                        break;
                    case STATE_TYPE_DOUBLE:
                        sin >> *(double*)ptr(se.offset); break;
                    case STATE_TYPE_STRING:
                        if (!consumeString(sin, (std::string*)ptr(se.offset))) {
                        	std::cout << "CONSUME STRING ERROR: " << (*this) << std::endl;
                       	return false;
                        }
                        break;
                    case STATE_TYPE_VECTOR2D:
                        sin >> IO( *(Eigen::Vector2d*)ptr(se.offset) ); break;
                    case STATE_TYPE_VECTOR3D:
                        sin >> IO( *(Eigen::Vector3d*)ptr(se.offset) ); break;
                    case STATE_TYPE_VECTORXD:
                        sin >> IO( *(Eigen::VectorXd*)ptr(se.offset) ); break;
                    case STATE_TYPE_CUSTOM:
                        if (!((State*)ptr(se.offset))->load(sin)) return false;
                        break;

                    default:
                        // std::cerr << "Invalid type: " << se.type << std::endl;
                        break;
                    }

                    char next = consumeTo(sin, '}', ',');
                    if (next == 0) {
                    	std::cout << "CONSUME END ERROR: " << (*this) << std::endl;
                    	return false;
                    }
                    if (next == '}') break;
                }
                

                return true;
            }

        };



        template <class T>
        struct StateArray : public AbstractState {
            StateArray() {
            }
            std::vector<T> elements;

            virtual void save(std::ostream& sout) const {
                sout << "[";
                for (typename std::vector<T>::const_iterator i = elements.begin();
                     i != elements.end(); i++) {
                    if (i != elements.begin()) {
                        sout << ", ";
                    }

                    const T& s = (*i);
                    s.save(sout);
                }
                sout << "]";
            }

            virtual bool load(std::istream& sin) {
                using namespace std;
                if (!consumeTo(sin, '[')) return false;
                clear();
                while(true) {
                    T s;
                    if (!(s.load(sin))) {
                    	cout << "Failed to load" << s << endl;
                    	return false;
                    }
                    push_back(s);

                    char next = consumeTo(sin, ']', ',');
//                    cout << "CLASS " << s << " : NEXT =" << next << endl;
                    if (next == 0) return false;
                    if (next == ']') break;
                }
                return true;

            }

            inline int getSize() const { return size(); }
            inline int size() { return elements.size(); }

            inline T& get(size_t i) { return elements.get(i); }
            inline const T& get(size_t i) const { return elements.get(i); }
            inline T& getLast() { return elements[size() - 1]; }
            inline const T& getLast() const { return elements[size() - 1]; }
            T& operator[](size_t i){ return elements[i];}
            const T& operator[](size_t i)const{ return elements[i];}


            void clear() { elements.clear(); }
            void push_back(const T& s) {
                elements.push_back(s);
            }
            void pop_back() {
                elements.pop_back();
            }
            
        };


    } // namespace toolkit
} // namespace rtql8



#endif // #ifndef TOOLKIT_STATE_H
