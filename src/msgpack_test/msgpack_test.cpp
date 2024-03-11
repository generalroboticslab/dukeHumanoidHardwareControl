#include <iostream>
#include <msgpack.hpp>
#include <vector>
#include <string>

class Person {
public:
    std::string name;
    int age;
    std::vector<std::string> hobbies;
// Tell MsgPack how to serialize/deserialize Person objects
    MSGPACK_DEFINE_MAP(name, age, hobbies); 
};



int main() {
    // Create a Person object
    Person person{"Alice", 25, {"reading", "hiking", "coding"}};

    // Serialize using MsgPack
    msgpack::sbuffer buffer; 
    msgpack::pack(buffer, person);

    // Deserialize back into a Person object
    msgpack::object_handle oh = msgpack::unpack(buffer.data(), buffer.size());
    msgpack::object obj = oh.get();

    Person deserialized_person;
    obj.convert(deserialized_person); 

    // Check the results
    std::cout << "Name: " << deserialized_person.name << std::endl;
    std::cout << "Age: " << deserialized_person.age << std::endl;
    std::cout << "Hobbies: ";
    for (const auto& hobby : deserialized_person.hobbies) {
        std::cout << hobby << " ";
    }
    std::cout << std::endl;

    return 0;
}