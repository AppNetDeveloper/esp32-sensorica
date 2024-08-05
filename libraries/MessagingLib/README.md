## Complex Arduino Serial Communication Made Manageable

![Messaging library classes.](https://www.cohesivecomputing.co.uk/cms/wp-content/uploads/2023/11/distributed-arduino-application.png "Distributed Arduino applications using the Messaging library.")

The Messaging library implements a communication protocol that is similar to Serialization found in frameworks such as .Net. It allows developers to convert simple data structures into a text format that is suitable for transmission over serial communication. This ensures interoperability between different platforms, such as Arduino to another MCU, or Arduino to Windows/Mac/Linux/Android applications. The latter platforms will require their own implementation of the MessageListener class and supporting abstract classes.

In summary, use the Messaging library to implement event driven (duplex) communication over serial between two Arduinos.

### Dependencies

The Messaging library depends on the following libraries:

- [StringLib](https://github.com/kashif-baig/StringLib) – efficient string building and reading

### Arduino Example Projects

Watch the video below that demonstrates communication between an Wemos D1 mini acting as a web server, and an Arduino Uno animating addressable RGB LED strip lights.

[![Arduino RGB LED UI](https://img.youtube.com/vi/1efcNaSA5K4/0.jpg)](https://www.youtube.com/watch?v=1efcNaSA5K4)

The Messaging library is covered in detail with examples in the ebook [Arduino Web Development: Pushing the Limits](https://www.cohesivecomputing.co.uk/arduino-web-development-pushing-the-limits/).
