## Parallax Spinneret Web Server

The **Parallax Spinneret Web Server** is an Ethernet-based development board designed for the Propeller microcontroller, enabling embedded web server capabilities for custom electronics projects.

**Overview**

- The Spinneret Web Server allows users to host web pages, serve files, and log data directly from embedded hardware.
    
- It is based on the Parallax Propeller microcontroller and uses a WIZnet Ethernet controller (W5100).
    
- Web content, files, and logs can be stored on a microSD card attached to the board[4](https://code.google.com/archive/p/spinneret-web-server)[1](https://www.mouser.com/ds/2/321/32203-Spinneret-Web-Server-Documentation-v1.1-709220.pdf).
    

## Key Features

- **Microcontroller**: [[Parallax Propeller 1]] (multi-core, programmable in Spin, C, or assembly)
    
- **Ethernet**: WIZnet W5100 controller for 10/100 Mbps connectivity
    
- **Storage**: microSD card slot for file and log storage
    
- **I/O**: Multiple general-purpose I/O pins for sensors, relays, and other peripherals
    
- **Web Server**: Can serve static HTML, XML, and other files; supports HTTP methods like GET and PUT[1](https://www.mouser.com/ds/2/321/32203-Spinneret-Web-Server-Documentation-v1.1-709220.pdf)[8](https://forums.parallax.com/discussion/149819/spinneret-msrobots)
    
- **Programming**: Supports firmware updates and web content uploads via network using HTTP PUT, among other methods[8](https://forums.parallax.com/discussion/149819/spinneret-msrobots)
    
- **Community Support**: Documentation, example projects, and active (though aging) forums[1](https://www.mouser.com/ds/2/321/32203-Spinneret-Web-Server-Documentation-v1.1-709220.pdf)[2](https://forums.parallax.com/discussion/166062/is-nobody-playing-with-spinneret-webserver-anymore)

## Documentation & Resources

- **Official Documentation**:  
    [Spinneret Web Server Documentation (PDF)](https://www.mouser.com/ds/2/321/32203-Spinneret-Web-Server-Documentation-v1.1-709220.pdf)[1](https://www.mouser.com/ds/2/321/32203-Spinneret-Web-Server-Documentation-v1.1-709220.pdf)
    
- **Google Code Archive**:  
    [Spinneret Web Server Project](https://code.google.com/archive/p/spinneret-web-server)[4](https://code.google.com/archive/p/spinneret-web-server)
    
- **Parallax Forums**:  
    [Spinneret Web Server Discussions](https://forums.parallax.com/discussion/166062/is-nobody-playing-with-spinneret-webserver-anymore)[2](https://forums.parallax.com/discussion/166062/is-nobody-playing-with-spinneret-webserver-anymore)
    
- **Propeller W5200/W5100 Driver Repository**:  
    [Spinneret-msrobots Documentation](http://parallax.msrobots.net/propeller-w5200-driver/Spinneret-msrobots/_README_Doc.htm)[8](https://forums.parallax.com/discussion/149819/spinneret-msrobots)
    
- **YouTube Preview**:  
    [Parallax Spinneret Web Server Preview](https://www.youtube.com/watch?v=4bFLyeYm2os)7
    

## Example Use Cases

- **Home Automation**: Remote monitoring and control of sensors and devices via web interface[2](https://forums.parallax.com/discussion/166062/is-nobody-playing-with-spinneret-webserver-anymore)
    
- **Industrial Monitoring**: Data logging and real-time status reporting for equipment and environmental sensors
    
- **Educational Projects**: Teaching embedded systems, networking, and web technologies
    

## Software & Firmware

- Supports custom web server software, with open-source projects available for advanced HTTP functionality (e.g., PUT, DELETE, MKCOL, OPTIONS handlers)[8](https://forums.parallax.com/discussion/149819/spinneret-msrobots).
    
- Firmware can be updated over the network, and files can be uploaded to the SD card using command-line tools like `curl`[8](https://forums.parallax.com/discussion/149819/spinneret-msrobots).
    
- Compatible with Propeller programming tools and libraries.
    

## Community & Status

- The Spinneret Web Server is no longer a flagship product and may be discontinued or replaced by newer solutions like ESP8266-based boards[2](https://forums.parallax.com/discussion/166062/is-nobody-playing-with-spinneret-webserver-anymore).
    
- However, a dedicated user base continues to maintain code, drivers, and documentation, especially for legacy and industrial applications[2](https://forums.parallax.com/discussion/166062/is-nobody-playing-with-spinneret-webserver-anymore)[8](https://forums.parallax.com/discussion/149819/spinneret-msrobots).
    
- Parallax's forums and code repositories remain valuable resources for troubleshooting and collaboration[2](https://forums.parallax.com/discussion/166062/is-nobody-playing-with-spinneret-webserver-anymore)[8](https://forums.parallax.com/discussion/149819/spinneret-msrobots).
    

## Related Links

- [Parallax, Inc. - Official Site](https://www.parallax.com/)[6](https://www.parallax.com/)
    
- [Parallax, Inc. - Wikipedia](https://en.wikipedia.org/wiki/Parallax,_Inc.)[3](https://en.wikipedia.org/wiki/Parallax,_Inc.)
    
- [Spinneret (Disambiguation) - Wikipedia](https://en.wikipedia.org/wiki/Spinneret_\(disambiguation\))[5](https://en.wikipedia.org/wiki/Spinneret_\(disambiguation\))
    

## Summary Table

|Feature|Details|
|---|---|
|Microcontroller|Parallax Propeller|
|Ethernet Controller|WIZnet W5100|
|Storage|microSD card|
|Web Server Capability|Static/dynamic content, HTTP methods|
|I/O|Multiple GPIO pins|
|Community Support|Parallax forums, code archives|
|Status|Legacy product, community maintained|

> For detailed technical documentation, see the [Spinneret Web Server Documentation (PDF)](https://www.mouser.com/ds/2/321/32203-Spinneret-Web-Server-Documentation-v1.1-709220.pdf)[1](https://www.mouser.com/ds/2/321/32203-Spinneret-Web-Server-Documentation-v1.1-709220.pdf).

_Note: If you are looking for new hardware, consider more recent alternatives like ESP8266/ESP32, but for Propeller-based or legacy projects, the Spinneret remains a powerful and flexible solution._

### Citations:

1. [https://www.mouser.com/ds/2/321/32203-Spinneret-Web-Server-Documentation-v1.1-709220.pdf](https://www.mouser.com/ds/2/321/32203-Spinneret-Web-Server-Documentation-v1.1-709220.pdf)
2. [https://forums.parallax.com/discussion/166062/is-nobody-playing-with-spinneret-webserver-anymore](https://forums.parallax.com/discussion/166062/is-nobody-playing-with-spinneret-webserver-anymore)
3. [https://en.wikipedia.org/wiki/Parallax,_Inc.](https://en.wikipedia.org/wiki/Parallax,_Inc.)
4. [https://code.google.com/archive/p/spinneret-web-server](https://code.google.com/archive/p/spinneret-web-server)
5. [https://en.wikipedia.org/wiki/Spinneret_(disambiguation)](https://en.wikipedia.org/wiki/Spinneret_\(disambiguation\))
6. [https://www.parallax.com](https://www.parallax.com/)
7. [https://www.youtube.com/watch?v=4bFLyeYm2os](https://www.youtube.com/watch?v=4bFLyeYm2os)
8. [https://forums.parallax.com/discussion/149819/spinneret-msrobots](https://forums.parallax.com/discussion/149819/spinneret-msrobots)

---
