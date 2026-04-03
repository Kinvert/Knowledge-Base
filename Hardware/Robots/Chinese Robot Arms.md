# Chinese Robot Arms

The Chinese robotics industry has undergone explosive growth since 2015, fundamentally disrupting the global market for robotic arms. Where industrial robots once cost $50,000-$150,000 from established players like [[FANUC]], [[ABB]], [[Yaskawa]], and [[KUKA]], Chinese manufacturers now offer comparable specifications at 20-40% of the price. This commoditization extends from industrial cobots down to hobbyist-grade desktop arms, creating unprecedented accessibility for small businesses, researchers, and makers.

**Why it matters**: A machine shop can now automate a CNC loading task for $8,000 instead of $45,000. A university robotics lab can equip 10 stations for what one [[Universal Robots]] arm would cost. Hobbyists can experiment with 6-DOF manipulation for under $500.

---

## 📊 Ultra-Budget Arms ($100-$1,000)

These arms target education, hobbyists, and light automation prototyping. Expect plastic construction, hobby servos, and limited payload/precision.

| Model | DOF | Payload | Reach | Price | Software | ROS | Notes |
|-------|-----|---------|-------|-------|----------|-----|-------|
| Elephant myCobot 280 | 6 | 250g | 280mm | $599 | myStudio | ✅ | M5Stack based, ESP32 |
| Elephant myCobot 280 Pi | 6 | 250g | 280mm | $699 | myStudio | ✅ | Built-in Raspberry Pi 4 |
| Elephant mechArm 270 | 6 | 250g | 270mm | $499 | myStudio | ✅ | Compact desktop |
| Hiwonder ArmPi Pro | 6 | 200g | 320mm | $399 | Python | ⚠️ | Vision kit included |
| Hiwonder xArm 1S | 6 | 500g | 380mm | $289 | LeArm | ❌ | LX-15D servos |
| Yahboom DOFBOT | 6 | 300g | 350mm | $289 | Python | ✅ | Jetson Nano compatible |
| LewanSoul xArm | 6 | 500g | 370mm | $199 | LeArm | ❌ | Popular starter arm |
| Waveshare RoArm-M2-S | 5 | 300g | 300mm | $229 | Python | ✅ | ESP32-S3 based |
| Freenove Robot Arm Kit | 6 | 200g | 320mm | $99 | Python | ❌ | DIY kit, learning focus |
| [[SO-ARM100]] | 6 | 300g | 300mm | $150-250 | LeRobot | ✅ | Open source, AI focus |

**Common servo types in this tier**: STS3215, LX-16A, LX-15D, MG996R, DS3218

---

## 📊 Budget Arms ($1,000-$5,000)

This tier offers metal construction, better servos or small BLDC motors, and starts to become viable for light production tasks.

| Model | DOF | Payload | Reach | Price | Software | ROS | Notes |
|-------|-----|---------|-------|-------|----------|-----|-------|
| UFACTORY xArm 5 Lite | 5 | 3kg | 700mm | $3,999 | xArm Studio | ✅ | Entry to xArm line |
| Elephant myCobot Pro 630 | 6 | 2kg | 630mm | $4,299 | myStudio Pro | ✅ | Industrial-lite |
| Dobot Magician | 4 | 500g | 320mm | $1,599 | DobotStudio | ✅ | Educational standard |
| Dobot MG400 | 4 | 750g | 440mm | $2,999 | DobotStudio | ✅ | Desktop SCARA |
| Hitbot Z-Arm S922 | 4 | 2kg | 600mm | $2,800 | Hitbot Studio | ✅ | SCARA, open-source SDK |
| WLKATA Mirobot | 6 | 150g | 260mm | $1,500 | WLkata Studio | ✅ | Industrial design, tiny |
| Annin AR4 (kit) | 6 | 1.5kg | 600mm | $2,500 | AR4 Software | ✅ | Open source, DIY |
| Trossen WidowX 250 | 6 | 250g | 650mm | $2,695 | Dynamixel SDK | ✅ | Interbotix line |
| Trossen ViperX 300 | 6 | 750g | 700mm | $4,495 | Dynamixel SDK | ✅ | Dynamixel X-series |

---

## 📊 Mid-Range Arms ($5,000-$15,000)

Proper cobots with industrial-grade harmonic drives, precision encoders, and production-ready software.

| Model | DOF | Payload | Reach | Repeatability | Price | Software |
|-------|-----|---------|-------|---------------|-------|----------|
| UFACTORY [[xArm 6]] | 6 | 5kg | 700mm | ±0.1mm | $8,990 | xArm Studio |
| UFACTORY xArm 7 | 7 | 3.5kg | 700mm | ±0.1mm | $11,999 | xArm Studio |
| Dobot CR3 | 6 | 3kg | 795mm | ±0.02mm | $6,000 | DobotStudio Pro |
| Dobot CR5 | 6 | 5kg | 910mm | ±0.02mm | $8,500 | DobotStudio Pro |
| AUBO i3 | 6 | 3kg | 625mm | ±0.02mm | $5,500 | AUBO Studio |
| AUBO i5 | 6 | 5kg | 886mm | ±0.02mm | $7,500 | AUBO Studio |
| JAKA Zu 3 | 6 | 3kg | 626mm | ±0.03mm | $5,000 | JAKA Studio |
| JAKA Zu 5 | 6 | 5kg | 800mm | ±0.03mm | $7,000 | JAKA Studio |
| Han's Elfin E03 | 6 | 3kg | 590mm | ±0.03mm | $6,000 | Elfin Studio |
| Han's Elfin E05 | 6 | 5kg | 800mm | ±0.02mm | $8,000 | Elfin Studio |
| Elite EC66 | 6 | 6kg | 914mm | ±0.03mm | $8,500 | Elite Studio |
| Fairino FR3 | 6 | 3kg | 560mm | ±0.02mm | $4,500 | Fairino Studio |
| Fairino FR5 | 6 | 5kg | 900mm | ±0.02mm | $6,000 | Fairino Studio |

---

## 📊 Industrial Cobots ($15,000-$40,000)

Heavy payload arms competing directly with UR10, UR16, and UR20 class robots.

| Brand | Model | Payload | Reach | Repeatability | Safety | Price Est |
|-------|-------|---------|-------|---------------|--------|-----------|
| AUBO | i10 | 10kg | 1350mm | ±0.02mm | ISO 10218 | $12,000-18,000 |
| AUBO | i16 | 16kg | 970mm | ±0.02mm | ISO 10218 | $15,000-22,000 |
| AUBO | i20 | 20kg | 1750mm | ±0.02mm | ISO 10218 | $18,000-28,000 |
| Dobot | CR10 | 10kg | 1300mm | ±0.02mm | ISO 10218 | $12,000-16,000 |
| Dobot | CR16 | 16kg | 1100mm | ±0.02mm | ISO 10218 | $15,000-20,000 |
| JAKA | Zu 12 | 12kg | 1327mm | ±0.03mm | ISO 10218 | $14,000-18,000 |
| JAKA | Zu 18 | 18kg | 1073mm | ±0.03mm | ISO 10218 | $18,000-24,000 |
| Han's | Elfin E10 | 10kg | 1000mm | ±0.02mm | ISO 10218 | $14,000-18,000 |
| Elite | EC612 | 12kg | 1304mm | ±0.03mm | ISO 10218 | $14,000-18,000 |
| Elite | EC620 | 20kg | 1800mm | ±0.03mm | ISO 10218 | $20,000-28,000 |
| Fairino | FR10 | 10kg | 1300mm | ±0.02mm | ISO 10218 | $8,000-14,000 |
| Fairino | FR16 | 16kg | 1000mm | ±0.02mm | ISO 10218 | $12,000-18,000 |
| Fairino | FR20 | 20kg | 1700mm | ±0.02mm | ISO 10218 | $15,000-22,000 |

---

## 📊 Major Chinese Manufacturers Overview

| Brand | HQ | Founded | Units Sold | Payload Range | Key Strength |
|-------|----|---------|-----------:|---------------|--------------|
| AUBO | Beijing | 2015 | 35,000+ | 3-20kg | Market leader, reliability |
| Dobot | Shenzhen | 2015 | 100,000+ | 0.5-16kg | Education to industrial |
| UFACTORY | Shenzhen | 2014 | 15,000+ | 3.5-12kg | ROS ecosystem, dev-friendly |
| Han's Robot | Shenzhen | 2017 | 25,000+ | 3-25kg | Parent is Han's Laser |
| JAKA | Shanghai | 2014 | 20,000+ | 3-18kg | Strong Japan/Korea presence |
| Elite Robot | Suzhou | 2014 | 18,000+ | 3-20kg | In-house components |
| Fairino | Beijing | 2016 | 12,000+ | 3-25kg | Aggressive pricing |
| Hitbot | Shenzhen | 2019 | 8,000+ | 1-5kg | SCARA specialist |
| Elephant | Shenzhen | 2016 | 50,000+ | 0.25-2kg | myCobot/education |
| Flexiv | Shanghai | 2016 | 3,000+ | 4-10kg | Adaptive force control |
| Rokae | Beijing | 2015 | 8,000+ | 3-20kg | Automotive focus |

---

## 🔄 Western vs Chinese Equivalents

This is the "what replaces what" section. Chinese arms now compete directly with established Western and Japanese manufacturers at dramatically lower price points.

### Universal Robots Replacements

| UR Model | Payload | Reach | UR Price | Chinese Alternative | Alt Price | Savings |
|----------|---------|-------|----------|---------------------|-----------|---------|
| UR3e | 3kg | 500mm | $23,000-28,000 | AUBO i3, JAKA Zu 3, Fairino FR3 | $4,000-6,000 | ~75-80% |
| UR5e | 5kg | 850mm | $30,000-45,000 | AUBO i5, Dobot CR5, JAKA Zu 5 | $6,000-10,000 | ~70-80% |
| UR10e | 12.5kg | 1300mm | $45,000-55,000 | AUBO i10, Dobot CR10, Elite EC612 | $10,000-18,000 | ~65-75% |
| UR16e | 16kg | 900mm | $50,000-65,000 | AUBO i16, Fairino FR16, JAKA Zu 18 | $12,000-22,000 | ~65-75% |
| UR20 | 20kg | 1750mm | $70,000-85,000 | Fairino FR20, Elite EC620, AUBO i20 | $15,000-28,000 | ~65-75% |
| UR30 | 30kg | 1300mm | $80,000-95,000 | Han's Elfin E25, Fairino FR25 | $25,000-40,000 | ~55-65% |

### FANUC Replacements

| FANUC Model | Payload | Reach | Est. Price | Chinese Alternative | Notes |
|-------------|---------|-------|------------|---------------------|-------|
| LR Mate 200iD/7L | 7kg | 911mm | $30,000-45,000 | xArm 6, Dobot CR5 | Desktop/cell |
| M-10iD/12 | 12kg | 1441mm | $45,000-65,000 | Dobot CR10, AUBO i10 | General handling |
| M-20iD/25 | 25kg | 1831mm | $55,000-85,000 | Fairino FR20, Elite EC620 | Heavy payload |
| CRX-10iA | 10kg | 1249mm | $40,000-55,000 | Han's Elfin E10, AUBO i10 | Cobot class |
| CRX-10iA/L | 10kg | 1418mm | $45,000-60,000 | Elite EC612, JAKA Zu 12 | Extended reach |
| CRX-20iA/L | 20kg | 1418mm | $55,000-70,000 | AUBO i20, Fairino FR20 | Heavy cobot |
| CRX-25iA | 25kg | 1889mm | $65,000-80,000 | Han's Elfin E25, Rokae xMate | Heavy cobot |

### ABB Replacements

| ABB Model | Payload | Reach | Est. Price | Chinese Alternative | Notes |
|-----------|---------|-------|------------|---------------------|-------|
| IRB 1200-5/0.9 | 5kg | 901mm | $35,000-50,000 | xArm 6, Dobot CR5 | Compact cell |
| IRB 1200-7/0.7 | 7kg | 700mm | $35,000-50,000 | UFACTORY xArm 7, AUBO i7 | Compact heavy |
| IRB 2600-12/1.65 | 12kg | 1650mm | $50,000-70,000 | AUBO i10, Dobot CR10 | Arc welding, handling |
| IRB 4600-45/2.05 | 45kg | 2050mm | $70,000-100,000 | ESTUN, Siasun (traditional) | Heavy industrial |
| GoFa CRB 10 | 10kg | 1620mm | $45,000-60,000 | Dobot CR10, JAKA Zu 12 | Cobot |
| GoFa CRB 15000 | 5kg | 950mm | $40,000-55,000 | Dobot CR5, JAKA Zu 5 | Cobot |
| SWIFTI CRB 1100 | 4kg | 580mm | $35,000-45,000 | xArm 5, AUBO i3 | Fast SCARA-like |
| YuMi IRB 14000 | 0.5kg | 559mm | $50,000-70,000 | Elephant myBuddy 280 | Dual-arm (partial) |

### Yaskawa Replacements

| Yaskawa Model | Payload | Reach | Est. Price | Chinese Alternative | Notes |
|---------------|---------|-------|------------|---------------------|-------|
| GP7 | 7kg | 927mm | $35,000-50,000 | xArm 6, JAKA Zu 7 | High speed handling |
| GP12 | 12kg | 1440mm | $45,000-60,000 | Dobot CR10, AUBO i10 | Machine tending |
| GP25 | 25kg | 1730mm | $55,000-75,000 | Fairino FR20, Elite EC620 | Heavy payload |
| GP50 | 50kg | 2061mm | $70,000-95,000 | ESTUN, Siasun | Traditional industrial |
| HC10 | 10kg | 1200mm | $45,000-60,000 | Han's Elfin E10, Fairino FR10 | Cobot |
| HC20 | 20kg | 1700mm | $55,000-75,000 | Fairino FR20, AUBO i20 | Heavy cobot |
| HC30PL | 30kg | 1700mm | $65,000-85,000 | Han's Elfin E25, Rokae | Palletizing cobot |

### KUKA Replacements

| KUKA Model | Payload | Reach | Est. Price | Chinese Alternative | Notes |
|------------|---------|-------|------------|---------------------|-------|
| KR 6 R900 | 6kg | 901mm | $40,000-55,000 | Dobot CR5, AUBO i5 | Compact |
| KR 10 R1100 | 10kg | 1101mm | $45,000-60,000 | AUBO i10, JAKA Zu 12 | General purpose |
| KR 16 R2010 | 16kg | 2010mm | $55,000-75,000 | Fairino FR16, AUBO i16 | Extended reach |
| LBR iiwa 7 | 7kg | 800mm | $70,000-100,000 | Flexiv Rizon 4 | 7-DOF, torque sensors |
| LBR iiwa 14 | 14kg | 820mm | $80,000-120,000 | Flexiv Rizon 10, Rokae xMate | 7-DOF, torque sensors |
| LBR iisy 3 | 3kg | 760mm | $35,000-50,000 | JAKA Zu 3, AUBO i3 | Entry cobot |
| LBR iisy 11 | 11kg | 1100mm | $50,000-65,000 | AUBO i10, Dobot CR10 | Mid cobot |
| LBR iisy 15 | 15kg | 930mm | $55,000-70,000 | AUBO i16, Fairino FR16 | Heavy cobot |

---

## ⚠️ What You Lose Going Chinese

| Aspect | Big 4 + UR | Chinese Alternative | Real Impact |
|--------|------------|---------------------|-------------|
| **Repeatability** | ±0.01-0.02mm typical | ±0.02-0.05mm typical | Minor for most applications |
| **Global Support** | 24/7, on-site in hours | Varies wildly, often days/weeks | **Major for production** |
| **Software Ecosystem** | Mature, massive community | Growing, smaller | Medium impact |
| **Safety Certifications** | Full CE, UL, ISO, TUV | Often CE only | **Check your requirements** |
| **Spare Parts** | Immediate from local distributor | Days-weeks from China | **Plan maintenance inventory** |
| **Cycle Time** | Often fastest in class | 10-20% slower typical | Depends on application |
| **Proven Longevity** | 10-15+ year lifespan documented | 5-8 years typical expectation | Long-term unclear |
| **Resale Value** | Strong used market | Very limited | Consider if upgrading often |
| **Documentation** | Comprehensive, multi-language | Variable quality, often Chinese-first | Integration challenges |
| **Training** | Extensive programs, local | Limited, online-focused | DIY learning curve |
| **Integration Ecosystem** | Huge third-party support | Growing but limited | More custom work needed |

### When Chinese Arms Make Sense

- R&D and prototyping (lower stakes)
- Educational institutions (price per station)
- Low-duty cycle applications
- Non-critical production backup
- Startups validating automation concepts
- Applications where you have in-house robotics expertise

### When to Stick with Established Brands

- 24/7 production lines
- Safety-critical applications
- Regulated industries (medical, pharma, food with strict requirements)
- No in-house robotics expertise
- Long support contracts required
- Integration with existing Big 4 fleet

---

## 📊 Software Ecosystem Comparison

| Brand | Proprietary SW | ROS1 | ROS2 | MoveIt | Python SDK | Blockly/Visual | Simulation |
|-------|----------------|------|------|--------|------------|----------------|------------|
| AUBO | AUBO Studio | ✅ | ✅ | ✅ | ✅ | ✅ | Gazebo |
| Dobot | DobotStudio Pro | ✅ | ✅ | ✅ | ✅ | ✅ | Coppeliasim |
| UFACTORY | xArm Studio | ✅ | ✅ | ✅ | ✅ | ✅ | Gazebo |
| Han's | Elfin Studio | ✅ | ⚠️ | ⚠️ | ✅ | ✅ | Custom |
| JAKA | JAKA Studio | ✅ | ⚠️ | ⚠️ | ✅ | ✅ | Custom |
| Elite | Elite Studio | ✅ | ⚠️ | ⚠️ | ✅ | ✅ | Custom |
| Fairino | Fairino Studio | ✅ | ⚠️ | ⚠️ | ✅ | ✅ | Custom |
| Hitbot | Hitbot Studio | ✅ | ✅ | ⚠️ | ✅ | ✅ | Gazebo |
| Elephant | myStudio | ✅ | ✅ | ✅ | ✅ | ✅ | Gazebo |
| Flexiv | Flexiv Elements | ✅ | ✅ | ✅ | ✅ | ✅ | Isaac Sim |

**Legend**: ✅ = Full support, ⚠️ = Partial/Community, ❌ = Not available

---

## 📊 Open Source / DIY Arms

| Project | DOF | Payload | Cost | Difficulty | Key Feature |
|---------|-----|---------|------|------------|-------------|
| [[SO-ARM100]] | 6 | 300g | $150-250 | Medium | LeRobot/HuggingFace integration |
| SO-ARM101 | 6 | 500g | $200-300 | Medium | Upgraded SO-ARM100 |
| Annin AR4 | 6 | 1.5kg | $2,000-3,000 | Hard | Stepper motors, industrial-lite |
| Annin AR3 | 6 | 1kg | $1,500-2,500 | Hard | Predecessor to AR4 |
| BCN3D Moveo | 5 | 500g | $500-800 | Medium | 3D printed, stepper based |
| Thor | 6 | 750g | $600-1,000 | Hard | 3D printed, harmonic drives |
| Niryo One/Ned | 6 | 500g | $600-1,800 | Easy | Full kit available |
| PAROL6 | 6 | 1kg | $800-1,200 | Medium | Source Motion design |
| OpenManipulator-X | 4-5 | 500g | $1,200+ | Medium | ROBOTIS/Dynamixel |

### Notable Open Source Resources

- **LeRobot** (HuggingFace): AI/ML framework for low-cost arms
- **MoveIt**: Motion planning for any ROS-compatible arm
- **ros2_control**: Hardware abstraction for custom arms
- **Annin Robotics**: Full build documentation and software

---

## 📊 DIY Actuator Options

Building your own arm? Here are common servo and actuator choices:

### Hobby Servos (Ultra-Budget)

| Brand | Model | Torque | Voltage | Protocol | Price | Notes |
|-------|-------|--------|---------|----------|-------|-------|
| Feetech | STS3215 | 19kg-cm | 6-8.4V | TTL Serial | $15-20 | SO-ARM100 standard |
| Feetech | SCS0009 | 1.5kg-cm | 4.8-6V | TTL Serial | $8-12 | Small joints |
| Feetech | STS3032 | 32kg-cm | 6-8.4V | TTL Serial | $25-30 | Higher torque |
| Hiwonder | LX-16A | 17kg-cm | 6-8.4V | TTL Serial | $12-18 | Budget daisy-chain |
| Hiwonder | LX-224 | 20kg-cm | 6-8.4V | TTL Serial | $18-25 | Higher duty cycle |
| Waveshare | ST3215 | 19kg-cm | 6-8.4V | TTL Serial | $18-22 | STS3215 clone |

### Smart Servos (Mid-Range)

| Brand | Model | Torque | Voltage | Protocol | Price | Notes |
|-------|-------|--------|---------|----------|-------|-------|
| Dynamixel | XL330-M288 | 0.52Nm | 5V | TTL/RS485 | $24 | Tiny, smart |
| Dynamixel | XL430-W250 | 1.4Nm | 11.1V | TTL | $50 | Popular mid-range |
| Dynamixel | XM430-W350 | 4.1Nm | 11.1-14.8V | RS485 | $220 | Higher torque |
| Dynamixel | XM540-W270 | 10.6Nm | 11.1-14.8V | RS485 | $300 | Research grade |
| Feetech | SMS30 | 30kg-cm | 6-8.4V | TTL | $35-45 | Magnetic encoder |

### BLDC Actuators (High Performance)

| Brand | Model | Torque | Voltage | Protocol | Price | Notes |
|-------|-------|--------|---------|----------|-------|-------|
| MyActuator | RMD-X6 | 6Nm | 24-48V | CAN/RS485 | $150-200 | Integrated driver |
| MyActuator | RMD-X8 | 9Nm | 24-48V | CAN/RS485 | $200-280 | Popular for arms |
| MyActuator | RMD-X8 Pro | 12Nm | 24-48V | CAN/RS485 | $280-350 | Higher torque |
| MyActuator | RMD-L | 2-6Nm | 24-48V | CAN/RS485 | $100-180 | Flat pancake |
| T-Motor | AK60-6 | 6Nm | 24V | CAN | $300-400 | MIT Mini Cheetah heritage |
| T-Motor | AK80-9 | 18Nm | 24-48V | CAN | $450-550 | High torque density |
| CubeMars | AK60-6 | 6Nm | 24V | CAN | $250-350 | T-Motor competitor |
| Gyems | RMD-X series | 3-12Nm | 24-48V | CAN/RS485 | $120-300 | MyActuator alternative |

### Gearbox Options

| Type | Ratio Range | Backlash | Efficiency | Price | Use Case |
|------|-------------|----------|------------|-------|----------|
| [[Harmonic Drive]] | 30:1-160:1 | <1 arcmin | 65-85% | $300-2000 | Precision joints |
| Planetary | 3:1-100:1 | 5-15 arcmin | 85-95% | $50-300 | General purpose |
| Cycloidal | 6:1-119:1 | 1-3 arcmin | 75-90% | $150-800 | High torque |
| Strain Wave (Chinese) | 50:1-100:1 | 1-3 arcmin | 65-80% | $80-400 | Budget harmonic alternative |

---

## 🏭 Major Manufacturer Profiles

### AUBO Robotics (遨博)

**Background**: Founded 2015 in Beijing, AUBO has become China's largest cobot manufacturer with over 35,000 units deployed globally. The name stands for "Autonomous Unified Bionic Operations."

**Product Line**:
- **i-series**: i3 (3kg), i5 (5kg), i7 (7kg), i10 (10kg), i12 (12kg), i16 (16kg), i20 (20kg)
- **C-series**: Compact versions for tight spaces

**Strengths**:
- Most mature Chinese cobot platform
- Extensive distributor network (including US)
- Strong ROS/ROS2 support
- Good documentation and training materials
- Proven reliability in production environments

**Weaknesses**:
- Higher priced than some Chinese competitors
- Software less polished than UR equivalent
- Limited vision system integration

**Where to Buy**: Direct, Dorna Robotics (US), various regional distributors

---

### Dobot

**Background**: Founded 2015 in Shenzhen, Dobot started with educational arms (Magician) and expanded to industrial cobots (CR series). Strong focus on the education market with over 100,000 units sold across all products.

**Product Line**:
- **Magician**: 4-DOF educational arm ($1,599)
- **Magician Lite**: Lower cost Magician ($599)
- **MG400**: Desktop SCARA ($2,999)
- **M1 Pro**: Larger SCARA
- **CR3/CR5/CR7/CR10/CR12/CR16**: Industrial cobots
- **Nova**: Next-gen cobot line

**Strengths**:
- Excellent educational materials and curriculum
- DobotStudio is user-friendly
- Wide product range from $600 to $20,000
- Strong MoveIt/ROS2 support for CR series
- US warehouse and support (Dobot Inc.)

**Weaknesses**:
- Magician is limited (4-DOF, tiny payload)
- CR series software less mature than i-series
- Some models have slower speed limits

**Where to Buy**: dobot.cc, Amazon, educational distributors

---

### UFACTORY (xArm)

**Background**: Founded 2014 in Shenzhen as UFactory, creators of the uArm (now discontinued) and xArm series. Strong focus on developers and researchers with excellent ROS integration.

**Product Line**:
- **xArm 5 Lite**: 5-DOF, 3kg payload ($3,999)
- **[[xArm 6]]**: 6-DOF, 5kg payload ($8,990)
- **xArm 7**: 7-DOF, 3.5kg payload ($11,999)
- **UF850**: 6-DOF, 5kg, 850mm reach
- **xArm 1S**: Low-cost educational (being phased out)

**Strengths**:
- Best ROS/ROS2/MoveIt integration of any Chinese arm
- Excellent Python SDK
- Active GitHub presence and developer community
- Good for research applications
- Direct sales model keeps prices competitive

**Weaknesses**:
- Limited to max 5kg payload
- No larger industrial models
- Smaller company, less support infrastructure

**Where to Buy**: ufactory.cc (direct), limited distributors

---

### Han's Robot (大族机器人)

**Background**: Subsidiary of Han's Laser (one of China's largest laser equipment manufacturers), founded 2017. Leverages parent company's precision manufacturing expertise.

**Product Line**:
- **Elfin series**: E03, E05, E10, E15, E25 (3-25kg)
- **Elfin Pro**: Enhanced versions
- **Elfin-P**: Palletizing specific

**Strengths**:
- Backed by major industrial conglomerate
- Wide payload range up to 25kg
- Competitive pricing
- Good build quality from precision manufacturing heritage

**Weaknesses**:
- Less developer-friendly than xArm/AUBO
- ROS support is community-driven
- Limited US presence

**Where to Buy**: han-s.com, regional distributors

---

### JAKA Robotics (节卡)

**Background**: Founded 2014 in Shanghai, JAKA has strong presence in Japan and Korea where they've displaced some UR installations. Focus on easy programming and small footprint.

**Product Line**:
- **Zu series**: Zu 3, Zu 5, Zu 7, Zu 12, Zu 18 (3-18kg)
- **MiniCobo**: Compact collaborative arm
- **All-in-One**: Controller integrated into base

**Strengths**:
- Very compact base design
- Strong East Asian market presence
- All-in-one design reduces integration complexity
- Good repeatability specs

**Weaknesses**:
- Limited US distribution
- Documentation often Chinese-first
- Smaller community

**Where to Buy**: jaka.com, Asian distributors

---

### Elite Robot (艾利特)

**Background**: Founded 2014 in Suzhou, Elite manufactures most components in-house including harmonic drives, giving them cost and supply chain advantages.

**Product Line**:
- **EC series**: EC63, EC66, EC612, EC614, EC620 (3-20kg)
- **CS series**: Traditional industrial variants

**Strengths**:
- Vertical integration keeps costs low
- Competitive specs
- In-house harmonic drives
- Growing US presence

**Weaknesses**:
- Smaller market share
- Less mature software ecosystem
- Limited training/support materials

**Where to Buy**: elibot.cn, emerging US distributors

---

### Fairino (法奥)

**Background**: Founded 2016 in Beijing, Fairino positions itself as providing "1/4 the price of competition" for equivalent specs. Very aggressive pricing strategy.

**Product Line**:
- **FR series**: FR3, FR5, FR10, FR16, FR20, FR25 (3-25kg)

**Strengths**:
- Most aggressive pricing in the market
- Full payload range
- Growing quickly

**Weaknesses**:
- Newer company, less proven
- Limited global support
- Software maturity questions

**Where to Buy**: fairino.com, limited distributors

---

### Hitbot (慧灵科技)

**Background**: Founded 2019 in Shenzhen, Hitbot specializes in SCARA and compact arms with open-source-friendly approaches.

**Product Line**:
- **Z-Arm series**: SCARA-style arms
- **S922, S1122, S1520**: Various reach/payload combos

**Strengths**:
- Open-source SDK and ROS support
- SCARA specialist
- Good developer documentation
- Competitive SCARA pricing

**Weaknesses**:
- SCARA-focused (limited 6-DOF options)
- Smaller company
- Limited to lighter payloads

**Where to Buy**: hitbotrobot.com

---

### Elephant Robotics (大象机器人)

**Background**: Founded 2016 in Shenzhen, Elephant targets the education and hobbyist market with myCobot series. Over 50,000 myCobot units sold.

**Product Line**:
- **myCobot 280**: 6-DOF, 250g, ESP32/M5Stack ($599)
- **myCobot 280 Pi**: With Raspberry Pi ($699)
- **myCobot 280 Jetson Nano**: With Jetson ($999)
- **mechArm 270**: 6-DOF compact ($499)
- **myBuddy 280**: Dual-arm ($1,399)
- **myArm**: Higher payload series
- **myPalletizer**: Palletizing arm
- **myCobot Pro 630**: Industrial-lite ($4,299)

**Strengths**:
- Best hobbyist/education ecosystem
- Extensive tutorials and projects
- M5Stack integration is clever
- Active community
- ROS/ROS2 support

**Weaknesses**:
- Very light payloads (250g typical)
- Plastic construction on low-end models
- Not for production use

**Where to Buy**: elephantrobotics.com, Amazon, Seeed Studio

---

### Flexiv (非夕)

**Background**: Founded 2016 in Shanghai with Silicon Valley R&D, Flexiv focuses on adaptive force control and AI integration. Premium positioning among Chinese manufacturers.

**Product Line**:
- **Rizon 4**: 4kg payload, adaptive force control
- **Rizon 4s**: Enhanced version
- **Rizon 10**: 10kg payload

**Strengths**:
- Best force control of any Chinese arm
- 7-DOF torque-controlled (like KUKA iiwa)
- Strong AI/ML integration
- High-end specs and build quality

**Weaknesses**:
- Most expensive Chinese option ($15,000-40,000)
- Limited model range
- Smaller company

**Where to Buy**: flexiv.com, limited direct sales

---

## 🛒 Where to Buy

### Direct from Manufacturer

| Manufacturer | Website | Ships to US | Lead Time |
|--------------|---------|-------------|-----------|
| AUBO | aubo-cobot.com | Via distributor | 2-4 weeks |
| Dobot | dobot.cc | ✅ Direct | 1-2 weeks |
| UFACTORY | ufactory.cc | ✅ Direct | 1-2 weeks |
| Han's Robot | han-s.com | Via distributor | 2-4 weeks |
| JAKA | jaka.com | Via distributor | 2-4 weeks |
| Elite | elibot.cn | Via distributor | 2-4 weeks |
| Fairino | fairino.com | Via distributor | 3-6 weeks |
| Hitbot | hitbotrobot.com | ✅ Direct | 2-3 weeks |
| Elephant | elephantrobotics.com | ✅ Direct | 1-2 weeks |

### US Distributors

- **Dorna Robotics**: AUBO distributor, California-based
- **Dobot Inc.**: Official Dobot US subsidiary
- **Top3DShop**: Various Chinese arms
- **Generation Robots**: European, ships to US
- **RobotShop**: Limited selection
- **Amazon**: Elephant, Hiwonder, Yahboom

### Alibaba/Made-in-China Tips

- Verify supplier has "Trade Assurance"
- Request video of specific unit before shipping
- Negotiate warranty terms explicitly
- Budget for import duties (7-25% for robots)
- Consider using inspection service
- Get detailed spec sheets, don't trust listings

### Red Flags

- No trade assurance or verified status
- Significantly below market pricing
- No response to technical questions
- Stock photos only, no factory images
- No clear warranty terms
- New seller with no transaction history

---

## 💰 Hidden Costs

Don't just budget for the arm. Real deployment costs:

### End Effectors

| Type | Price Range | Notes |
|------|-------------|-------|
| Basic gripper | $200-800 | 2-finger parallel |
| Vacuum gripper | $300-1,500 | Depends on complexity |
| Electric gripper | $500-2,000 | Robotiq, OnRobot alternatives |
| Custom gripper | $1,000-5,000 | 3D printed or machined |
| Tool changer | $2,000-8,000 | Automatic tool switching |

### Vision Systems

| Type | Price Range | Notes |
|------|-------------|-------|
| Basic 2D camera | $200-500 | OpenCV compatible |
| Depth camera (RealSense) | $300-600 | Intel D435/D455 |
| Industrial 2D | $1,000-3,000 | Basler, FLIR |
| 3D structured light | $2,000-8,000 | Photoneo, Zivid |
| Integrated vision kits | $1,500-5,000 | Manufacturer solutions |

### Safety Equipment

| Item | Price Range | Notes |
|------|-------------|-------|
| Safety scanner | $1,500-4,000 | SICK, Keyence, OMRON |
| Light curtain | $500-2,000 | Perimeter protection |
| Emergency stop system | $200-500 | Required for production |
| Safety-rated PLC | $1,000-3,000 | If needed for integration |
| Fencing/guarding | $500-5,000 | Traditional safeguarding |

### Integration

| Item | Price Range | Notes |
|------|-------------|-------|
| Mounting/table | $200-2,000 | Stable mounting critical |
| Cable management | $100-500 | Drag chains, etc. |
| Controller cabinet | $200-1,000 | If external controller |
| Integration labor | $5,000-50,000+ | Highly variable |
| Programming/commissioning | $2,000-20,000+ | Depends on complexity |

### Ongoing Costs

| Item | Frequency | Cost |
|------|-----------|------|
| Harmonic drive replacement | Every 10,000-20,000 hrs | $500-2,000 |
| Belt replacement | Every 5,000-10,000 hrs | $50-200 |
| Calibration | Annually | $500-2,000 |
| Software updates | Varies | Sometimes subscription |
| Spare parts inventory | Upfront | $500-5,000 |

---

## 📖 Specs Glossary

### Payload

The maximum mass the arm can handle at its end effector. Important caveats:
- Rated at specific wrist orientation (usually wrist down)
- Includes end effector weight (gripper counts against payload)
- Reduces with extended reach
- Different at different speeds

### Repeatability vs Accuracy

- **Repeatability**: How precisely the arm returns to the same point (±0.02mm typical for cobots)
- **Accuracy**: How close to the commanded position the arm actually goes
- Repeatability is always tighter than accuracy
- Most tasks only need repeatability (teaching points empirically)

### Degrees of Freedom (DOF)

| DOF | Capabilities |
|-----|--------------|
| 3 | Point-to-point, no orientation control |
| 4 | SCARA-like, fixed tool angle |
| 5 | Most orientations, some singularities |
| 6 | Full orientation control, standard for manipulation |
| 7 | Redundant, can avoid obstacles while maintaining pose |

### IP Ratings

| Rating | Protection | Typical Use |
|--------|------------|-------------|
| IP20 | No water protection | Clean rooms only |
| IP54 | Dust protected, splash resistant | General industrial |
| IP65 | Dust tight, water jet protected | Wet environments |
| IP67 | Dust tight, submersion protected | Washdown areas |

### Cobot vs Industrial

| Aspect | Cobot | Industrial |
|--------|-------|------------|
| Safety | Power/force limiting | Requires fencing |
| Speed | 1-2 m/s typical | 2-5 m/s typical |
| Payload | Usually <25kg | Up to 1000kg+ |
| Programming | Teach pendant, easy | Often requires integrator |
| Cost | Lower | Higher |
| Flexibility | Easy to redeploy | Fixed installation |

---

## 🔗 Related Concepts

- [[SO-ARM100]] - Open source arm with LeRobot integration
- [[Unitree]] - Chinese robotics (quadrupeds)
- [[Robot Kits]] - DIY robotics
- [[Servo Motor]] - Standard hobby servos
- [[BLDC]] - Brushless DC motors
- [[Harmonic Drive]] - Precision gearbox technology
- [[FOC]] - Field Oriented Control for motors
- [[ROS2]] - Robot Operating System 2
- [[MoveIt]] - Motion planning framework
- [[Electric Motors]] - Motor fundamentals
- [[Inverse Kinematics]] - Arm motion calculation
- [[PID Controller]] - Basic control theory
- [[CAN]] - Common actuator protocol
- [[RS-485]] - Serial communication for servos
- [[Stepper Motor]] - Alternative to servos

---

## 📚 External Resources

### Manufacturer Links

- [AUBO Robotics](https://www.aubo-cobot.com/)
- [Dobot](https://www.dobot.cc/)
- [UFACTORY xArm](https://www.ufactory.cc/)
- [Han's Robot](https://www.han-s.com/)
- [JAKA Robotics](https://www.jaka.com/)
- [Elite Robot](https://www.elibot.cn/)
- [Fairino](https://www.fairino.com/)
- [Hitbot](https://www.hitbotrobot.com/)
- [Elephant Robotics](https://www.elephantrobotics.com/)
- [Flexiv](https://www.flexiv.com/)

### GitHub Repositories

- [xArm ROS2](https://github.com/xArm-Developer/xarm_ros2)
- [Dobot CR ROS2](https://github.com/Dobot-Arm/dobot_ros2)
- [AUBO ROS2](https://github.com/AuboRobot/aubo_ros2)
- [Elephant myCobot](https://github.com/elephantrobotics/mycobot_ros2)
- [LeRobot](https://github.com/huggingface/lerobot)
- [Annin Robotics AR4](https://github.com/SkyentificGit/AR4)

### Community Resources

- [r/robotics](https://reddit.com/r/robotics)
- [ROS Discourse](https://discourse.ros.org/)
- [Hackaday Robot Projects](https://hackaday.io/projects?tag=robot%20arm)

### Market Reports

- IFR (International Federation of Robotics) annual reports
- Interact Analysis cobot market tracking
- GGII (China) robotics industry data
