# International Atomic Time (TAI)

International Atomic Time (TAI, from the French _Temps Atomique International_) is the world's most precise time standard, maintained by averaging the output of hundreds of atomic clocks across the globe. TAI is foundational for scientific, technical, and civil applications that require extremely accurate and stable timekeeping.

## **What is TAI?**

- **Definition:** TAI is a continuous time scale based on the proper time on Earth's geoid, realized by a weighted average of over 450 atomic clocks in more than 80 national laboratories worldwide[1](https://en.wikipedia.org/wiki/International_Atomic_Time)[2](https://www.timeanddate.com/time/international-atomic-time.html)[4](https://www.britannica.com/science/International-Atomic-Time).
    
- **Precision:** Atomic clocks used for TAI deviate by only about 1 second in up to 100 million years[2](https://www.timeanddate.com/time/international-atomic-time.html).
    
- **Measurement:** The base unit, the second, is defined via the Cesium-133 atom: one second equals 9,192,631,770 transitions between two hyperfine levels of its ground state[2](https://www.timeanddate.com/time/international-atomic-time.html).
    

## **How TAI Works**

- **Continuous Scale:** TAI does not use leap seconds; every day is exactly 86,400 seconds long[1](https://en.wikipedia.org/wiki/International_Atomic_Time)[9](https://en.wikipedia.org/wiki/Unix_time).
    
- **Reference:** TAI is the principal realization of Terrestrial Time and is the basis for Coordinated Universal Time (UTC)[1](https://en.wikipedia.org/wiki/International_Atomic_Time)[2](https://www.timeanddate.com/time/international-atomic-time.html).
    
- **Offset from UTC:** As of January 2017, UTC is 37 seconds behind TAI due to the addition of leap seconds to UTC to keep it in sync with Earth's rotation[1](https://en.wikipedia.org/wiki/International_Atomic_Time)[2](https://www.timeanddate.com/time/international-atomic-time.html).
    

## **Standards and Protocols Using TAI**

- **Coordinated Universal Time (UTC):** UTC is derived from TAI but includes leap seconds to stay aligned with Earth's rotation[1](https://en.wikipedia.org/wiki/International_Atomic_Time)[2](https://www.timeanddate.com/time/international-atomic-time.html).
    
- **Terrestrial Time (TT):** TAI serves as the basis for TT, used in astronomy and space science[1](https://en.wikipedia.org/wiki/International_Atomic_Time).
    
- **Network Time Protocol (NTP):** While NTP typically distributes UTC, its internal calculations and some implementations may reference TAI for precision[6](https://www.nist.gov/document/sp432-02pdf).
    
- **GNSS (Global Navigation Satellite Systems):** Systems like GPS, Galileo, and others use time scales closely related to TAI for synchronization and navigation[3](https://gssc.esa.int/navipedia/index.php/Atomic_Time).
    
- [[PTP]] uses TAI, and [[XTSS]] also supports it.
## **Comparisons: TAI vs. UTC vs. Unix Time**

|Feature|TAI|UTC|Unix Time|
|---|---|---|---|
|Basis|Atomic clocks (no leap seconds)|TAI + leap seconds (to match Earth's rotation)|Seconds since 1970-01-01 00:00:00 UTC|
|Leap Seconds|None|Yes|Not handled directly; some ambiguity at leap sec|
|Day Length|Always 86,400 seconds|Usually 86,400 seconds; 86,401 with leap sec|Always 86,400 seconds|
|Use Case|Scientific, technical|Civil timekeeping, global synchronization|Computing, file timestamps|
|Offset (as of 2025)|Reference|37 seconds behind TAI|Matches UTC except at leap seconds|
|Example Epoch|1970-01-01 00:00:00 TAI|1970-01-01 00:00:00 UTC|0 (Unix epoch)|

> **Note:** TAI fits the description of an idealized time scale better than UTC or Unix time, as it does not have discontinuities or ambiguities caused by leap seconds[7](https://github.com/qntm/t-a-i)[9](https://en.wikipedia.org/wiki/Unix_time).

## **Why Not Use TAI for Everything?**

- **TAI's precision is ideal for scientific and technical applications**, but it does not account for the Earth's variable rotation, which is why civil timekeeping uses UTC with leap seconds[2](https://www.timeanddate.com/time/international-atomic-time.html).
    
- **Leap seconds in UTC** ensure that noon remains close to the time the sun is at its highest point in the sky, maintaining the link between civil time and solar time[2](https://www.timeanddate.com/time/international-atomic-time.html)[9](https://en.wikipedia.org/wiki/Unix_time).
    

## **Further Reading and Sources**

- [Wikipedia: International Atomic Time](https://en.wikipedia.org/wiki/International_Automatic_Time)
    
- [Time and Date: International Atomic Time](https://www.timeanddate.com/time/international-atomic-time.html)
    
- [Britannica: International Atomic Time](https://www.britannica.com/science/International-Atomic-Time)
    
- [Navipedia: Atomic Time](https://gssc.esa.int/navipedia/index.php/Atomic_Time)
    
- [Unix time and TAI comparison](https://en.wikipedia.org/wiki/Unix_time)
    
- [NIST: Time and Frequency Services (PDF)](https://www.nist.gov/document/sp432-02pdf)
    

## **Summary**

- **TAI** is the most stable and precise time standard, essential for science and technology.
    
- **UTC** is derived from TAI, adding leap seconds for civil use.
    
- **Unix time** is widely used in computing, based on UTC but does not handle leap seconds directly.
    
- **TAI is the foundation** for many global standards and protocols, even if not directly visible in everyday life.
    

For further technical details, see the linked sources above.

### Citations:

1. [https://en.wikipedia.org/wiki/International_Atomic_Time](https://en.wikipedia.org/wiki/International_Atomic_Time)
2. [https://www.timeanddate.com/time/international-atomic-time.html](https://www.timeanddate.com/time/international-atomic-time.html)
3. [https://gssc.esa.int/navipedia/index.php/Atomic_Time](https://gssc.esa.int/navipedia/index.php/Atomic_Time)
4. [https://www.britannica.com/science/International-Atomic-Time](https://www.britannica.com/science/International-Atomic-Time)
5. [https://www.dictionary.com/browse/international-atomic-time](https://www.dictionary.com/browse/international-atomic-time)
6. [https://www.nist.gov/document/sp432-02pdf](https://www.nist.gov/document/sp432-02pdf)
7. [https://github.com/qntm/t-a-i](https://github.com/qntm/t-a-i)
8. [https://ntrs.nasa.gov/api/citations/19750009116/downloads/19750009116.pdf](https://ntrs.nasa.gov/api/citations/19750009116/downloads/19750009116.pdf)
9. [https://en.wikipedia.org/wiki/Unix_time](https://en.wikipedia.org/wiki/Unix_time)

---

Answer from Perplexity: [https://www.perplexity.ai/search/please-write-me-a-good-clean-o-Y6xZBV4uSR.xwbi1wWE_UQ?utm_source=copy_output](https://www.perplexity.ai/search/please-write-me-a-good-clean-o-Y6xZBV4uSR.xwbi1wWE_UQ?utm_source=copy_output)