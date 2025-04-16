# Inventus Battery Status

# Header
std_msgs/Header header

# Battery Version and Serial Number
string hardware_version
string software_version
string firmware_version
string serial_number

# Battery Status Values
float32 current
float32 voltage
float32 temperature
float32 min_cell_temperature
float32 max_cell_temperature
float32 state_of_charge
float32 full_charge_capacity
float32 design_capacity
float32 remaining_capacity
float32[] cell_voltage

# Mode
uint16 op_mode

uint16 OP_MODE_MODULE_BALANCING = 1
uint16 OP_MODE_SHIP = 2
uint16 OP_MODE_PRE_DISCHARGE = 3
uint16 OP_MODE_STANDBY = 4
uint16 OP_MODE_DISCHARGE = 5
uint16 OP_MODE_CHARGE = 6
uint16 OP_MODE_FAULT = 7
uint16 OP_MODE_PRE_CHARGE = 8

# Charge Faults
uint16 charge_fault

uint16 CHARGE_FAULT_HIGH_TEMP = 1
uint16 CHARGE_FAULT_LOW_TEMP = 2
uint16 CHARGE_FAULT_OVER_CURRENT = 4
uint16 CHARGE_FAULT_OVER_VOLTAGE = 8
uint16 CHARGE_FAULT_SHORT_CIRCUIT = 16
uint16 CHARGE_FAULT_OTHER = 32
uint16 CHARGE_FAULT_MOSFET_TEMP = 64
uint16 CHARGE_FAULT_SEVERE_UNDER_VOLTAGE = 128
uint16 CHARGE_FAULT_COM_AFE_FAILED = 256
uint16 CHARGE_FAULT_HW_SECOND_OVP = 512             # Marked as reserved as of SW guide 1.4
uint16 CHARGE_FAULT_RESERVED = 1024
uint16 CHARGE_FAULT_PRECHARGE_FAILED = 2048         # Marked as reserved as of SW guide 1.4
uint16 CHARGE_FAULT_PACK_PARALLEL_ERROR = 4096      # Marked as reserved as of SW guide 1.4
uint16 CHARGE_FAULT_SIGNAL_ERROR = 8192             # Marked as reserved as of SW guide 1.4
uint16 CHARGE_FAULT_PREDISCHARGE_FAILED = 16384     # Marked as reserved as of SW guide 1.4
uint16 CHARGE_FAULT_INTERNAL_COMM_FAILURE = 32768

# Discharge Fault
uint16 discharge_fault

uint16 DISCHARGE_FAULT_HIGH_TEMP = 1
uint16 DISCHARGE_FAULT_LOW_TEMP = 2
uint16 DISCHARGE_FAULT_OVER_CURRENT = 4
uint16 DISCHARGE_FAULT_UNDER_VOLTAGE = 8
uint16 DISCHARGE_FAULT_SHORT_CIRCUIT = 16
uint16 DISCHARGE_FAULT_OTHER = 32
uint16 DISCHARGE_FAULT_MOSFET_TEMP = 64
uint16 DISCHARGE_FAULT_SEVERE_UNDER_VOLTAGE = 128
uint16 DISCHARGE_FAULT_COM_AFE_FAILED = 256
uint16 DISCHARGE_FAULT_HW_SECOND_OVP = 512          # Marked as reserved as of SW guide 1.4
uint16 DISCHARGE_FAULT_RESERVED = 1024
uint16 DISCHARGE_FAULT_RESERVED_2 = 2048
uint16 DISCHARGE_FAULT_PACK_PARALLEL_ERROR = 4096   # Marked as reserved as of SW guide 1.4
uint16 DISCHARGE_FAULT_SIGNAL_ERROR = 8192          # Marked as reserved as of SW guide 1.4
uint16 DISCHARGE_FAULT_PREDISCHARGE_FAILED = 16384  # Marked as reserved as of SW guide 1.4
uint16 DISCHARGE_FAULT_INTERNAL_COMM_FAILURE = 32768
