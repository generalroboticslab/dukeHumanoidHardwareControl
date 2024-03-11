#ifndef PERSON_H
#define PERSON_H

#include <string>
#include <vector>
#include <msgpack.hpp>

class Person {
public:
    std::string name;
    int age;
    // std::vector<std::string> hobbies;
    std::vector<double> hobbies;

    MSGPACK_DEFINE_MAP(name, age, hobbies); // For MsgPack serialization
};

#endif 
