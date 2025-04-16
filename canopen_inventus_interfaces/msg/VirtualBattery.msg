# Inventus Virtual Battery
# All PDO produced by the batteries refer to the multi-battery system.
# Therefore, this message must only be published by the master node.

# Header
std_msgs/Header header

# TPDO1
uint8 number_of_batteries           # Number of batteries in parallel.
float32 soc                         # Average SOC of all non-faulted batteries.
float32 sum_capacity                # Sum of the remaining capacity of all non-faulted batteries.
float32 remaining_runtime           # Remaining run time of all non-faulted batteries.
float32 remaining_chargetime        # Remaining time to full charge of all non-faulted batteries.

# TPDO2
float32 avg_voltage                 # Average voltage of all non-faulted battery packs.
float32 sum_current                 # Combined current value of all non-faulted battery packs.
                                    # Charge/regen current (+), Discharge current (-).
float32 discharge_current_limit     # Combined discharge current limit of all non-faulted battery packs.
float32 charge_cutoff_current       # Combined taper current of all non-faulted battery packs when nearing charge cutoff voltage.
uint8 fully_charged                 # 0 – All non-faulted battery packs not at charge termination voltage.
                                    # 1 – All non-faulted battery packs at charge termination voltage.

# TPDO3
float32 avg_temp                    # Average temperature of all non-faulted battery packs.
float32 discharge_cutoff_voltage    # Voltage at which a battery pack will open FETs to stop discharging.
float32 charge_current_limit        # Combined charge current limit of all non-faulted battery packs.
float32 max_voltage_allowed         # Maximum allowed charging voltage.

# TPDO4
float32 avg_health                  # Average state of health of all non-faulted battery packs.
uint8 num_faulted_batteries         # Number of batteries in the system that are faulted.
uint8 num_active_batteries          # Number of batteries in the system that are not faulted.
uint8 op_mode                       # Operational mode of all non-faulted battery packs.
                                    # This value will only show “fault mode” if all battery packs in the system are faulted.
uint16 charge_fault                 # Combined charge fault of all battery packs.
uint16 discharge_fault              # Combined discharge fault of all battery packs.

# TPD05
float32 regen_current_limit         # Combined regen current limit of all non-faulted battery packs.
float32 min_cell_voltage            # Min cell voltage of all non-faulted battery packs.
float32 max_cell_voltage            # Max cell voltage of all non-faulted battery packs.
uint8 cell_balance_status           # Cell balance status of all battery packs.
                                    # Each bit will represent one battery, nth bit will represent nth battery pack.
                                        # Bit = 1 means cell balance of this battery pack is enabling.
                                        # Bit = 0 means cell balance of this battery pack is Idle.

# TPD06
float32 all_avg_voltage             # Average pack voltage of all battery packs
float32 all_soc                     # Average SOC value of all battery packs
float32 all_avg_temp                # Average temperature of all battery packs
uint8 master_node_id                # The node ID of master pack

# Operational Model Enum
uint16 OP_MODE_MODULE_BALANCING = 1
uint16 OP_MODE_SHIP = 2
uint16 OP_MODE_PRE_DISCHARGE = 3
uint16 OP_MODE_STANDBY = 4
uint16 OP_MODE_DISCHARGE = 5
uint16 OP_MODE_CHARGE = 6
uint16 OP_MODE_FAULT = 7
uint16 OP_MODE_PRE_CHARGE = 8
uint16 avail_op_modes

# Charge Fault Bit Masks
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
uint16 avail_charge_faults

# Discharge Fault Bit Masks
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
uint16 avail_discharge_faults
