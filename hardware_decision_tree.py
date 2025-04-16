#!/usr/bin/env python3
# hardware_decision_tree.py

import pandas as pd
import numpy as np
from sklearn import tree
import matplotlib.pyplot as plt
import graphviz
from io import StringIO

# Create a structured dataset from the hardware specifications
def create_dataset():
    # Define all device characteristics based on technical documentation
    data = {
        # Microcontrollers
        'ESP32-WROOM-32': {
            'type': 'Microcontroller',
            'processing_power': 3,       # Scale 1-5 (1=low, 5=very high)
            'power_consumption': 2,      # Scale 1-5 (1=very low, 5=high)
            'wireless': 1,               # Boolean: 0=no, 1=yes
            'ethernet': 0,               # Boolean
            'analog_quality': 2,         # Scale 1-5 (1=none, 5=excellent)
            'memory_size': 2,            # Scale 1-5
            'real_time': 1,              # Scale 0-2 (0=none, 1=soft, 2=hard)
            'development_ease': 4,       # Scale 1-5 (1=complex, 5=very easy)
            'parallel_processing': 2,    # Scale 1-5
            'cost': 1,                   # Scale 1-5 (1=low, 5=high)
            'video_capability': 0,       # Boolean
            'linux_capable': 0,          # Boolean
        },
        'NUCLEO-F446RE': {
            'type': 'Microcontroller',
            'processing_power': 3,
            'power_consumption': 2,
            'wireless': 0,
            'ethernet': 0,
            'analog_quality': 4,
            'memory_size': 1,
            'real_time': 2,
            'development_ease': 3,
            'parallel_processing': 1,
            'cost': 1,
            'video_capability': 0,
            'linux_capable': 0,
        },
        'NUCLEO-F767ZI': {
            'type': 'Microcontroller',
            'processing_power': 4,
            'power_consumption': 2,
            'wireless': 0,
            'ethernet': 1,
            'analog_quality': 4,
            'memory_size': 2,
            'real_time': 2,
            'development_ease': 3,
            'parallel_processing': 1,
            'cost': 2,
            'video_capability': 1,
            'linux_capable': 0,
        },
        'MSP-EXP430FR5994': {
            'type': 'Microcontroller',
            'processing_power': 1,
            'power_consumption': 1,
            'wireless': 0,
            'ethernet': 0,
            'analog_quality': 2,
            'memory_size': 1,
            'real_time': 2,
            'development_ease': 4,
            'parallel_processing': 1,
            'cost': 1,
            'video_capability': 0,
            'linux_capable': 0,
        },
        'MSP-EXP432P401R': {
            'type': 'Microcontroller',
            'processing_power': 1,
            'power_consumption': 1,
            'wireless': 0,
            'ethernet': 0,
            'analog_quality': 5,
            'memory_size': 1,
            'real_time': 2,
            'development_ease': 4,
            'parallel_processing': 1,
            'cost': 1,
            'video_capability': 0,
            'linux_capable': 0,
        },
        'MSP-EXP432P4111': {
            'type': 'Microcontroller',
            'processing_power': 1,
            'power_consumption': 1,
            'wireless': 0,
            'ethernet': 0,
            'analog_quality': 5,
            'memory_size': 2,
            'real_time': 2,
            'development_ease': 3,
            'parallel_processing': 1,
            'cost': 1,
            'video_capability': 0,
            'linux_capable': 0,
        },

        # FPGA Boards
        'DE10-LITE': {
            'type': 'FPGA',
            'processing_power': 3,
            'power_consumption': 3,
            'wireless': 0,
            'ethernet': 0,
            'analog_quality': 2,
            'memory_size': 1,
            'real_time': 2,
            'development_ease': 1,
            'parallel_processing': 5,
            'cost': 2,
            'video_capability': 1,
            'linux_capable': 0,
        },
        'PYNQ-Z2': {
            'type': 'FPGA',
            'processing_power': 4,
            'power_consumption': 3,
            'wireless': 0,
            'ethernet': 1,
            'analog_quality': 2,
            'memory_size': 3,
            'real_time': 2,
            'development_ease': 2,
            'parallel_processing': 5,
            'cost': 3,
            'video_capability': 1,
            'linux_capable': 1,
        },
        'BAYS 3': {
            'type': 'FPGA',
            'processing_power': 3,
            'power_consumption': 3,
            'wireless': 0,
            'ethernet': 0,
            'analog_quality': 2,
            'memory_size': 1,
            'real_time': 2,
            'development_ease': 1,
            'parallel_processing': 5,
            'cost': 3,
            'video_capability': 1,
            'linux_capable': 0,
        },

        # Mini PCs
        'Raspberry Pi 5': {
            'type': 'Mini PC',
            'processing_power': 5,
            'power_consumption': 4,
            'wireless': 1,
            'ethernet': 1,
            'analog_quality': 1,
            'memory_size': 5,
            'real_time': 0,
            'development_ease': 5,
            'parallel_processing': 2,
            'cost': 2,
            'video_capability': 1,
            'linux_capable': 1,
        },
        'Raspberry Pi 4': {
            'type': 'Mini PC',
            'processing_power': 4,
            'power_consumption': 3,
            'wireless': 1,
            'ethernet': 1,
            'analog_quality': 1,
            'memory_size': 4,
            'real_time': 0,
            'development_ease': 5,
            'parallel_processing': 2,
            'cost': 2,
            'video_capability': 1,
            'linux_capable': 1,
        },
        'Raspberry Pi Zero 2W': {
            'type': 'Mini PC',
            'processing_power': 3,
            'power_consumption': 2,
            'wireless': 1,
            'ethernet': 0,
            'analog_quality': 1,
            'memory_size': 2,
            'real_time': 0,
            'development_ease': 5,
            'parallel_processing': 2,
            'cost': 1,
            'video_capability': 1,
            'linux_capable': 1,
        },
    }

    # Convert dictionary to DataFrame
    df = pd.DataFrame.from_dict(data, orient='index')
    return df

def train_decision_tree(df):
    # Features for decision making
    X = df.drop(['type'], axis=1)
    y = df.index  # The recommended device is the target

    # Create and train the decision tree classifier
    clf = tree.DecisionTreeClassifier(max_depth=5)
    clf = clf.fit(X, y)

    return clf, X.columns

def visualize_tree(clf, feature_names, class_names):
    dot_data = tree.export_graphviz(
        clf,
        out_file=None,
        feature_names=feature_names,
        class_names=class_names,
        filled=True,
        rounded=True,
        special_characters=True
    )
    graph = graphviz.Source(dot_data)
    return graph

def interactive_selector():
    """Interactive hardware selection based on project requirements"""

    # Dictionary mapping device types to their specific information
    device_info = {
        'ESP32-WROOM-32': {
            'description': 'Dual-core 240MHz microcontroller with Wi-Fi/BT',
            'strengths': [
                'Built-in Wi-Fi and Bluetooth',
                'Dual-core performance up to 240MHz',
                'Large community and support',
                'Low cost',
                'Good battery life with sleep modes'
            ],
            'ideal_for': [
                'IoT devices requiring wireless connectivity',
                'Smart home sensors and controllers',
                'Wireless data loggers',
                'Battery-powered devices with periodic connectivity',
                'Prototyping and hobby projects'
            ]
        },
        'NUCLEO-F446RE': {
            'description': '180MHz ARM Cortex-M4F MCU with rich peripherals',
            'strengths': [
                'Excellent real-time performance',
                'Multiple timers and communication interfaces',
                'DSP instructions and floating-point unit',
                'Precise ADCs (12-bit)',
                'Arduino shield compatibility'
            ],
            'ideal_for': [
                'Real-time control systems',
                'Precision sensing and control',
                'Motor control and robotics',
                'Industrial automation',
                'Wired communication interfaces'
            ]
        },
        'NUCLEO-F767ZI': {
            'description': '216MHz ARM Cortex-M7 MCU with Ethernet and advanced peripherals',
            'strengths': [
                'High-performance Cortex-M7 core',
                'Built-in Ethernet',
                'Camera interface and graphics acceleration',
                'Large flash and RAM',
                'Multiple high-speed interfaces'
            ],
            'ideal_for': [
                'High-performance embedded systems',
                'Industrial control with networking',
                'Camera/imaging applications',
                'User interfaces with graphics',
                'Complex real-time systems'
            ]
        },
        'MSP-EXP430FR5994': {
            'description': 'Ultra-low power 16MHz MCU with FRAM memory',
            'strengths': [
                'Ultra-low power consumption',
                'FRAM non-volatile memory',
                'LEA accelerator for signal processing',
                'Fast wake-up time',
                'Long battery life'
            ],
            'ideal_for': [
                'Ultra-low power sensor nodes',
                'Battery-operated devices (months/years)',
                'Energy harvesting applications',
                'Portable measurement equipment',
                'Data logging in power-constrained environments'
            ]
        },
        'MSP-EXP432P401R': {
            'description': 'Low-power 48MHz ARM Cortex-M4F with 14-bit ADC',
            'strengths': [
                'Ultra-low power ARM core',
                'High-precision 14-bit ADC',
                'Multiple low-power modes',
                'Good balance of performance and efficiency',
                'ARM ecosystem compatibility'
            ],
            'ideal_for': [
                'Low-power precision sensing',
                'Battery-operated measurement devices',
                'Portable medical or scientific equipment',
                'Secure IoT sensor nodes',
                'Long-term environmental monitoring'
            ]
        },
        'MSP-EXP432P4111': {
            'description': 'Low-power 48MHz ARM with 2MB flash, LCD driver, and USB',
            'strengths': [
                'Large memory (2MB flash, 256KB RAM)',
                'Integrated segment LCD driver',
                'USB device support',
                'Low-power ARM core',
                'High-precision 14-bit ADC'
            ],
            'ideal_for': [
                'Portable instruments with displays',
                'Low-power standalone systems',
                'Battery-operated measurement devices',
                'Medical or scientific equipment',
                'USB-connected sensing devices'
            ]
        },
        'DE10-LITE': {
            'description': 'Intel MAX 10 FPGA with integrated ADC and Arduino compatibility',
            'strengths': [
                'Reconfigurable digital logic',
                'Hardware parallelism',
                'Built-in ADC in MAX 10',
                'Arduino Uno shield compatibility',
                'VGA output and expansion headers'
            ],
            'ideal_for': [
                'Custom digital logic implementation',
                'Parallel data processing',
                'Hardware acceleration',
                'Digital signal processing',
                'Learning FPGA development'
            ]
        },
        'PYNQ-Z2': {
            'description': 'Xilinx Zynq SoC with dual-core ARM and FPGA fabric',
            'strengths': [
                'Python-programmable FPGA development',
                'Hybrid ARM + FPGA architecture',
                'HDMI In/Out and audio codec',
                'Ethernet and expansion headers',
                'Pre-built overlays available'
            ],
            'ideal_for': [
                'Hardware acceleration of algorithms',
                'Machine learning at the edge',
                'Video/image processing',
                'Teaching heterogeneous computing',
                'Prototyping novel computing architectures'
            ]
        },
        'BAYS 3': {
            'description': 'Xilinx Artix-7 FPGA educational board with rich I/O',
            'strengths': [
                'Educational focused design',
                'Rich onboard I/O (switches, LEDs, displays)',
                'VGA output',
                'Pmod expansion ports',
                'USB-HID host for keyboard/mouse'
            ],
            'ideal_for': [
                'Digital logic education',
                'Hardware design learning',
                'Custom digital circuits',
                'Hardware prototyping',
                'Simple video/graphics experiments'
            ]
        },
        'Raspberry Pi 5': {
            'description': '2.4GHz quad-core ARM A76 with PCIe, GPU, and rich I/O',
            'strengths': [
                'Powerful quad-core A76 processor',
                'VideoCore VII GPU with Vulkan support',
                'PCIe expansion slot',
                'Dual 4K HDMI outputs',
                'Comprehensive software ecosystem'
            ],
            'ideal_for': [
                'Edge computing devices',
                'Computer vision projects',
                'AI/ML at the edge',
                'Media centers/players',
                'Desktop replacement for basic tasks'
            ]
        },
        'Raspberry Pi 4': {
            'description': '1.5GHz quad-core ARM A72 with GPU and USB 3.0',
            'strengths': [
                'Quad-core Cortex-A72 processor',
                'Up to 8GB RAM',
                'Dual 4K HDMI outputs',
                'USB 3.0 and Gigabit Ethernet',
                'Vast software library and community'
            ],
            'ideal_for': [
                'Edge computing devices',
                'IoT gateways/hubs',
                'Media centers/players',
                'Web/database servers',
                'General Linux computing'
            ]
        },
        'Raspberry Pi Zero 2W': {
            'description': 'Compact 1GHz quad-core ARM A53 with Wi-Fi/BT',
            'strengths': [
                'Tiny form factor',
                'Quad-core A53 processor',
                'Built-in Wi-Fi and Bluetooth',
                'Low cost',
                'Pi compatibility in a smaller package'
            ],
            'ideal_for': [
                'Space-constrained projects',
                'Low-power IoT devices',
                'Portable projects',
                'Cost-sensitive applications',
                'Wireless control systems'
            ]
        }
    }

    print("\033[1;36m" + "="*80)
    print("HARDWARE SELECTION DECISION TREE")
    print("This tool will help you select the optimal hardware for your project")
    print("="*80 + "\033[0m")

    # Series of questions to determine the best device
    questions = [
        {
            'text': "What is your primary hardware requirement?",
            'options': [
                "Full operating system (Linux) with high processing power",
                "Custom digital logic and hardware parallelism",
                "Microcontroller for embedded applications"
            ]
        },
        {
            'text': "What is your power constraint?",
            'options': [
                "Ultra-low power (battery for months/years)",
                "Low power (battery for days/weeks)",
                "Standard power (regular charging or mains power)"
            ]
        },
        {
            'text': "Do you need wireless connectivity?",
            'options': [
                "Yes, Wi-Fi and/or Bluetooth is essential",
                "No, don't need wireless or will add external module"
            ]
        },
        {
            'text': "What level of analog interface do you need?",
            'options': [
                "High-precision analog (14-bit+ ADC)",
                "Standard analog (12-bit ADC)",
                "Basic/minimal analog",
                "No analog needed"
            ]
        },
        {
            'text': "Do you need real-time processing capabilities?",
            'options': [
                "Yes, hard real-time requirements",
                "Soft real-time is sufficient",
                "No real-time requirements"
            ]
        },
        {
            'text': "How important is development simplicity?",
            'options': [
                "Very important (prefer high-level languages/frameworks)",
                "Balanced (moderate learning curve acceptable)",
                "Not important (willing to learn complex tools)"
            ]
        },
        {
            'text': "What's your processing requirement?",
            'options': [
                "Very high (compute-intensive tasks, AI, etc.)",
                "High (complex algorithms, graphics)",
                "Medium (typical embedded applications)",
                "Low (simple sensing, control tasks)"
            ]
        }
    ]

    # Score system for each device
    scores = {device: 0 for device in device_info.keys()}

    # Process the first question to determine category
    print(f"\n\033[1;33m{questions[0]['text']}\033[0m")
    for i, option in enumerate(questions[0]['options'], 1):
        print(f"{i}. {option}")

    while True:
        try:
            choice = int(input("\nEnter your choice (number): "))
            if 1 <= choice <= len(questions[0]['options']):
                break
            print("Invalid choice. Please try again.")
        except ValueError:
            print("Please enter a number.")

    # Filter devices based on first choice
    filtered_devices = []

    if choice == 1:  # Linux/OS
        filtered_devices = ['Raspberry Pi 5', 'Raspberry Pi 4', 'Raspberry Pi Zero 2W', 'PYNQ-Z2']
    elif choice == 2:  # FPGA
        filtered_devices = ['DE10-LITE', 'PYNQ-Z2', 'BAYS 3']
    else:  # MCU
        filtered_devices = ['ESP32-WROOM-32', 'NUCLEO-F446RE', 'NUCLEO-F767ZI',
                           'MSP-EXP430FR5994', 'MSP-EXP432P401R', 'MSP-EXP432P4111']

    # Process remaining questions
    for q_index in range(1, len(questions)):
        q = questions[q_index]

        # Skip questions that aren't relevant for our filtered devices
        if q_index == 1 and choice == 1:  # Skip power question for SBCs
            continue
        if q_index == 4 and choice == 1:  # Skip real-time question for SBCs
            continue

        print(f"\n\033[1;33m{q['text']}\033[0m")
        for i, option in enumerate(q['options'], 1):
            print(f"{i}. {option}")

        while True:
            try:
                user_choice = int(input("\nEnter your choice (number): "))
                if 1 <= user_choice <= len(q['options']):
                    break
                print("Invalid choice. Please try again.")
            except ValueError:
                print("Please enter a number.")

        # Update scores based on answers
        for device in filtered_devices:
            # Power constraint scoring
            if q_index == 1:
                if user_choice == 1:  # Ultra-low power
                    if device in ['MSP-EXP430FR5994', 'MSP-EXP432P401R', 'MSP-EXP432P4111']:
                        scores[device] += 5
                    elif device in ['ESP32-WROOM-32', 'NUCLEO-F446RE', 'NUCLEO-F767ZI']:
                        scores[device] += 2
                    # FPGAs get 0
                elif user_choice == 2:  # Low power
                    if device in ['ESP32-WROOM-32', 'NUCLEO-F446RE']:
                        scores[device] += 5
                    elif device in ['MSP-EXP430FR5994', 'MSP-EXP432P401R', 'MSP-EXP432P4111']:
                        scores[device] += 4
                    elif device == 'NUCLEO-F767ZI':
                        scores[device] += 3
                    # FPGAs get lower scores
                    elif device in ['DE10-LITE', 'BAYS 3', 'PYNQ-Z2']:
                        scores[device] += 1
                else:  # Standard power
                    if device in ['NUCLEO-F767ZI', 'DE10-LITE', 'BAYS 3', 'PYNQ-Z2']:
                        scores[device] += 4
                    elif device in ['ESP32-WROOM-32', 'NUCLEO-F446RE']:
                        scores[device] += 3
                    # Ultra-low power MCUs get lower score for standard power
                    elif device in ['MSP-EXP430FR5994', 'MSP-EXP432P401R', 'MSP-EXP432P4111']:
                        scores[device] += 1

            # Wireless connectivity
            elif q_index == 2:
                if user_choice == 1:  # Wireless needed
                    if device == 'ESP32-WROOM-32':
                        scores[device] += 5
                    elif device in ['Raspberry Pi 5', 'Raspberry Pi 4', 'Raspberry Pi Zero 2W']:
                        scores[device] += 4
                    # Everything else gets 0
                else:  # Wireless not needed
                    if device not in ['ESP32-WROOM-32', 'Raspberry Pi 5', 'Raspberry Pi 4', 'Raspberry Pi Zero 2W']:
                        scores[device] += 2  # Small bonus for not needing wireless

            # Analog interface
            elif q_index == 3:
                if user_choice == 1:  # High-precision
                    if device in ['MSP-EXP432P401R', 'MSP-EXP432P4111']:  # 14-bit ADC
                        scores[device] += 5
                    elif device in ['NUCLEO-F446RE', 'NUCLEO-F767ZI']:  # 12-bit but good quality
                        scores[device] += 3
                    elif device in ['ESP32-WROOM-32', 'MSP-EXP430FR5994', 'DE10-LITE', 'PYNQ-Z2', 'BAYS 3']:
                        scores[device] += 2
                    # Raspberry Pis get 0 as they have no analog
                elif user_choice == 2:  # Standard analog
                    if device in ['NUCLEO-F446RE', 'NUCLEO-F767ZI']:
                        scores[device] += 5
                    elif device in ['ESP32-WROOM-32', 'MSP-EXP430FR5994', 'MSP-EXP432P401R', 'MSP-EXP432P4111']:
                        scores[device] += 3
                    elif device in ['DE10-LITE', 'PYNQ-Z2', 'BAYS 3']:
                        scores[device] += 2
                    # Raspberry Pis get 0
                elif user_choice == 3:  # Basic analog
                    if device in ['NUCLEO-F446RE', 'NUCLEO-F767ZI', 'ESP32-WROOM-32']:
                        scores[device] += 4
                    elif device in ['MSP-EXP430FR5994', 'MSP-EXP432P401R', 'MSP-EXP432P4111',
                                   'DE10-LITE', 'PYNQ-Z2', 'BAYS 3']:
                        scores[device] += 3
                    # Raspberry Pis get 0
                else:  # No analog needed
                    if device in ['Raspberry Pi 5', 'Raspberry Pi 4', 'Raspberry Pi Zero 2W']:
                        scores[device] += 3  # Bonus for RPis as they don't have analog anyway

            # Real-time processing
            elif q_index == 4:
                if user_choice == 1:  # Hard real-time
                    if device in ['NUCLEO-F446RE', 'NUCLEO-F767ZI', 'MSP-EXP430FR5994',
                                'MSP-EXP432P401R', 'MSP-EXP432P4111']:
                        scores[device] += 5
                    elif device in ['DE10-LITE', 'PYNQ-Z2', 'BAYS 3']:
                        scores[device] += 4  # FPGAs can do real-time with proper design
                    elif device == 'ESP32-WROOM-32':
                        scores[device] += 3  # ESP32 can do soft real-time
                    # Raspberry Pis get 0
                elif user_choice == 2:  # Soft real-time
                    if device == 'ESP32-WROOM-32':
                        scores[device] += 5
                    elif device in ['NUCLEO-F446RE', 'NUCLEO-F767ZI', 'MSP-EXP430FR5994',
                                   'MSP-EXP432P401R', 'MSP-EXP432P4111']:
                        scores[device] += 4
                    elif device in ['DE10-LITE', 'PYNQ-Z2', 'BAYS 3']:
                        scores[device] += 3
                    elif device in ['Raspberry Pi 5', 'Raspberry Pi 4', 'Raspberry Pi Zero 2W']:
                        scores[device] += 1  # RPis can do soft real-time with compromises
                else:  # No real-time
                    if device in ['Raspberry Pi 5', 'Raspberry Pi 4', 'Raspberry Pi Zero 2W', 'PYNQ-Z2']:
                        scores[device] += 4
                    elif device == 'ESP32-WROOM-32':
                        scores[device] += 3
                    # MCUs and FPGAs get lower scores as they're overkill for non-real-time

            # Development simplicity
            elif q_index == 5:
                if user_choice == 1:  # Very important
                    if device in ['Raspberry Pi 5', 'Raspberry Pi 4', 'Raspberry Pi Zero 2W']:
                        scores[device] += 5
                    elif device == 'ESP32-WROOM-32':
                        scores[device] += 4  # Arduino support
                    elif device in ['MSP-EXP430FR5994', 'MSP-EXP432P401R', 'MSP-EXP432P4111']:
                        scores[device] += 3  # Energia support
                    elif device in ['NUCLEO-F446RE', 'NUCLEO-F767ZI']:
                        scores[device] += 2
                    elif device == 'PYNQ-Z2':
                        scores[device] += 1  # Python but still complex
                    # Other FPGAs get 0
                elif user_choice == 2:  # Balanced
                    if device in ['ESP32-WROOM-32', 'NUCLEO-F446RE']:
                        scores[device] += 4
                    elif device in ['NUCLEO-F767ZI', 'MSP-EXP430FR5994', 'MSP-EXP432P401R', 'MSP-EXP432P4111']:
                        scores[device] += 3
                    elif device == 'PYNQ-Z2':
                        scores[device] += 2
                    elif device in ['DE10-LITE', 'BAYS 3']:
                        scores[device] += 1
                else:  # Not important
                    if device in ['DE10-LITE', 'BAYS 3', 'PYNQ-Z2']:
                        scores[device] += 5  # Reward FPGAs for complex users
                    elif device == 'NUCLEO-F767ZI':
                        scores[device] += 4  # Reward complex MCUs

            # Processing requirement
            elif q_index == 6:
                if user_choice == 1:  # Very high
                    if device == 'Raspberry Pi 5':
                        scores[device] += 5
                    elif device in ['Raspberry Pi 4', 'PYNQ-Z2']:
                        scores[device] += 4
                    elif device == 'Raspberry Pi Zero 2W':
                        scores[device] += 2
                    elif device == 'NUCLEO-F767ZI':
                        scores[device] += 1
                    # Everything else gets 0
                elif user_choice == 2:  # High
                    if device in ['Raspberry Pi 5', 'Raspberry Pi 4']:
                        scores[device] += 5
                    elif device in ['PYNQ-Z2', 'Raspberry Pi Zero 2W', 'NUCLEO-F767ZI']:
                        scores[device] += 4
                    elif device in ['DE10-LITE', 'BAYS 3', 'ESP32-WROOM-32']:
                        scores[device] += 3
                    elif device == 'NUCLEO-F446RE':
                        scores[device] += 2
                    # MSP gets 0
                elif user_choice == 3:  # Medium
                    if device in ['ESP32-WROOM-32', 'NUCLEO-F446RE', 'NUCLEO-F767ZI']:
                        scores[device] += 5
                    elif device in ['Raspberry Pi Zero 2W', 'DE10-LITE', 'BAYS 3']:
                        scores[device] += 4
                    elif device in ['MSP-EXP432P401R', 'MSP-EXP432P4111']:
                        scores[device] += 3
                    elif device == 'MSP-EXP430FR5994':
                        scores[device] += 2
                    # RPi 4/5 are overkill
                    elif device in ['Raspberry Pi 5', 'Raspberry Pi 4']:
                        scores[device] += 1
                else:  # Low
                    if device in ['MSP-EXP430FR5994', 'MSP-EXP432P401R', 'MSP-EXP432P4111']:
                        scores[device] += 5
                    elif device in ['ESP32-WROOM-32', 'NUCLEO-F446RE']:
                        scores[device] += 3
                    # Everything else is overkill and gets 0-1

    # Filter only the original filtered devices
    filtered_scores = {device: score for device, score in scores.items() if device in filtered_devices}

    # Sort by score
    sorted_devices = sorted(filtered_scores.items(), key=lambda x: x[1], reverse=True)

    # Show top 3 recommendations
    print("\n\033[1;32m" + "="*80)
    print("RECOMMENDED HARDWARE BASED ON YOUR REQUIREMENTS:")
    print("="*80 + "\033[0m")

    for i, (device, score) in enumerate(sorted_devices[:3], 1):
        print(f"\n\033[1;36m{i}. {device} (Score: {score})\033[0m")
        print(f"   {device_info[device]['description']}")
        print("\n   \033[1;33mKey Strengths:\033[0m")
        for strength in device_info[device]['strengths']:
            print(f"   ‚Ä¢ {strength}")
        print("\n   \033[1;33mIdeal Applications:\033[0m")
        for app in device_info[device]['ideal_for']:
            print(f"   ‚Ä¢ {app}")
        print()

    return sorted_devices[0][0]  # Return the top recommendation

if __name__ == "__main__":
    # Create dataset from hardware specifications
    df = create_dataset()

    # Train a decision tree model
    clf, feature_names = train_decision_tree(df)

    # Visualize the tree (would output a graphviz visualization)
    # For console application, we'll use an interactive approach instead
    recommended_device = interactive_selector()

    print("\n\033[1;33mThank you for using the Hardware Selection Tool!\033[0m")
