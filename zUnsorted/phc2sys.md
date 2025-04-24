
#zunsorted
# phc2sys

phc2sys is a Linux utility for synchronizing two or more clocks within a system. Its most common use is to synchronize the system clock (CLOCK_REALTIME) to a PTP (Precision Time Protocol) hardware clock (PHC), typically found on network interface cards (NICs). This process is essential for environments requiring precise timekeeping, such as finance, telecommunications, and industrial automation.

## **Purpose and Functionality**

- **Primary Role:**  
    Synchronize the system clock with a PTP hardware clock (PHC), which itself is usually synchronized to a network time source via the ptp4l daemon[1](https://linuxptp.nwtime.org/documentation/phc2sys/)[3](https://manpages.ubuntu.com/manpages/noble/man8/phc2sys.8.html)[4](https://docs.nvidia.com/networking/display/NVIDIA5TTechnologyUserManualv10/Synchronizing+System+Clock)[7](https://documentation.suse.com/sled/15-SP5/html/SLED-all/cha-tuning-ptp.html)[8](https://manpages.debian.org/unstable/linuxptp/phc2sys.8.en.html).
    
- **Why It Matters:**  
    Accurate time synchronization is critical for distributed systems, high-frequency trading, and other applications where even microsecond discrepancies can be problematic.
    

## **How phc2sys Works**

- **Clock Relationships:**
    
    - The PHC (on the NIC) acts as the master.
        
    - The system clock is the slave[7](https://documentation.suse.com/sled/15-SP5/html/SLED-all/cha-tuning-ptp.html).
        
    - ptp4l synchronizes the PHC to the network PTP grandmaster.
        
    - phc2sys ensures the system clock follows the PHC.
        
- **Synchronization Modes:**
    
    - **Automatic:** Using the `-a` option, phc2sys automatically fetches clocks from a running ptp4l instance and dynamically adjusts the synchronization direction based on PTP port states[3](https://manpages.ubuntu.com/manpages/noble/man8/phc2sys.8.html)[8](https://manpages.debian.org/unstable/linuxptp/phc2sys.8.en.html).
        
    - **Manual:** Specify source (`-s`) and sink (`-c`) clocks directly. Two modes are supported:
        
        - Using a PPS (Pulse Per Second) signal from the source clock.
            
        - Reading time directly from the source clock[3](https://manpages.ubuntu.com/manpages/noble/man8/phc2sys.8.html)[7](https://documentation.suse.com/sled/15-SP5/html/SLED-all/cha-tuning-ptp.html).
            

## **Key Options and Usage**

|Option|Description|
|---|---|
|`-a`|Automatic synchronization using ptp4l state|
|`-s <device>`|Specify source clock (e.g., NIC interface name)|
|`-c <device>`|Specify sink clock (e.g., system clock)|
|`-w`|Wait for ptp4l to synchronize before starting|
|`-O <offset>`|Set TAI-UTC offset manually (important if not waiting for ptp4l sync)|
|`-m`|Print log messages to standard output|
|`-u <seconds>`|Set statistics update interval|
|`-f <config>`|Use configuration file for options|
|`-r`|Reverse synchronization direction if host becomes PTP domain server|
|`--step_threshold`|Set maximum offset to correct by stepping the clock on first update|

## **Example Commands**

bash

`# Synchronize system clock to PHC on eth0, wait for ptp4l sync sudo phc2sys -s eth0 -w # Synchronize with manual TAI-UTC offset sudo phc2sys -s eth0 -O -35 # Run automatically with ptp4l, print logs, update stats every 60 seconds sudo phc2sys -a -m -u 60 # Start as a systemd service (reads options from /etc/sysconfig/phc2sys) sudo systemctl start phc2sys`

## **Service Integration**

- **Systemd Service:**  
    phc2sys can run as a background service managed by systemd. Configuration is typically read from `/etc/sysconfig/phc2sys`[2](https://docs.redhat.com/en/documentation/red_hat_enterprise_linux/6/html/deployment_guide/s1-synchronizing_the_clocks)[5](https://docs.redhat.com/en/documentation/red_hat_enterprise_linux/7/html/system_administrators_guide/ch-configuring_ptp_using_ptp4l)[7](https://documentation.suse.com/sled/15-SP5/html/SLED-all/cha-tuning-ptp.html).
    
- **Enable/Disable Service:**
    
    bash
    
    `sudo systemctl enable phc2sys sudo systemctl disable phc2sys`
    

## **Monitoring and Verification**

- **Output Example:**  
    phc2sys periodically logs time offsets, frequency adjustments, and path delays. Offsets below 100 ns indicate excellent synchronization[5](https://docs.redhat.com/en/documentation/red_hat_enterprise_linux/7/html/system_administrators_guide/ch-configuring_ptp_using_ptp4l)[9](https://tsn.readthedocs.io/timesync.html).
    
- **Sample Log Output:**
    
    text
    
    `phc2sys[528.528]: phc offset 55341 s0 freq +0 delay 2729 phc2sys[533.528]: phc offset -73 s2 freq -37026 delay 2764 phc2sys[700.948]: rms 1837 max 10123 freq -36474 ± 4752 delay 2752 ± 16`
    
    - _rms_: Root mean square offset
        
    - _max_: Maximum absolute offset
        
    - _freq_: Frequency adjustment (mean ± stddev)
        
    - _delay_: Path delay (mean ± stddev)[5](https://docs.redhat.com/en/documentation/red_hat_enterprise_linux/7/html/system_administrators_guide/ch-configuring_ptp_using_ptp4l)
        

## **Advanced Configuration**

- **TAI vs UTC:**  
    PTP operates in International Atomic Time (TAI), while the system clock uses Coordinated Universal Time (UTC). Use the `-O` option to specify the offset if not waiting for ptp4l[7](https://documentation.suse.com/sled/15-SP5/html/SLED-all/cha-tuning-ptp.html).
    
- **Manual and Automated Modes:**  
    phc2sys can be used in both manual and automated environments, supporting complex topologies and multi-interface systems[6](https://docs.starlingx.io/system_configuration/kubernetes/ptp-instance-examples-517dce312f56.html).
    

## **Typical Workflow**

1. Start ptp4l to synchronize the PHC to the network.
    
2. Start phc2sys to synchronize the system clock to the PHC.
    
3. Monitor logs to ensure offsets are within acceptable bounds.
    

## **References**

- [phc2sys official documentation](https://linuxptp.nwtime.org/documentation/phc2sys/)
    
- [Red Hat and SUSE guides](https://docs.redhat.com/en/documentation/red_hat_enterprise_linux/6/html/deployment_guide/s1-synchronizing_the_clocks)[5](https://docs.redhat.com/en/documentation/red_hat_enterprise_linux/7/html/system_administrators_guide/ch-configuring_ptp_using_ptp4l)[7](https://documentation.suse.com/sled/15-SP5/html/SLED-all/cha-tuning-ptp.html)
    
- [Debian & Ubuntu manpages](https://manpages.ubuntu.com/manpages/noble/man8/phc2sys.8.html)[8](https://manpages.debian.org/unstable/linuxptp/phc2sys.8.en.html)
    

## **See Also**

- [[ptp4l]]
    
- [[PTP]] Precision Time Protocol
    
- [[Network Time Synchronization]]