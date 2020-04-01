import queue
import json
import numpy as np
from rtls_util import RtlsUtil, RtlsUtilLoggingLevel, RtlsUtilException

logging_file = "log.txt"
rtlsUtil = RtlsUtil(logging_file, RtlsUtilLoggingLevel.ALL)

devices = [
    {"com_port": "COM73", "baud_rate": 460800, "name": "CC26x2 Master"},
    {"com_port": "COM97", "baud_rate": 460800, "name": "CC26x2 Passive"}
]
## Setup devices
master_node, passive_nodes, all_nodes = rtlsUtil.set_devices(devices)
rtlsUtil.reset_devices()
print("Devices Reset")
try:
    scan_results = rtlsUtil.scan(15)
except:
    print("Slave не обнаружен. Запустите программу снова")
    exit()
rtlsUtil.ble_connect(scan_results[0], 100)
print("Connection Success")    

aoa_params = {
                "aoa_run_mode": "AOA_MODE_ANGLE",  ## AOA_MODE_ANGLE, AOA_MODE_PAIR_ANGLES, AOA_MODE_RAW
                "aoa_cc2640r2": {
                "aoa_cte_scan_ovs": 4,
                "aoa_cte_offset": 4,
                "aoa_cte_length": 20,
                "aoa_sampling_control": int('0x00', 16),
            },
                "aoa_cc26x2": {
                "aoa_slot_durations": 1,
                "aoa_sample_rate": 1,
                "aoa_sample_size": 1,
                "aoa_sampling_control": int('0x10', 16),
                ## bit 0   - 0x00 - default filtering, 0x01 - RAW_RF no filtering,
                ## bit 4,5 - default: 0x10 - ONLY_ANT_1, optional: 0x20 - ONLY_ANT_2
                "aoa_sampling_enable": 1,
                "aoa_pattern_len": 2,
                "aoa_ant_pattern": [0, 1]
    }
}
try:
    rtlsUtil.aoa_set_params(aoa_params)
except:
    print("Не удалось установить параметры конфигурации. Запустите снова")
print("AOA Paramas Set")

rtlsUtil.aoa_start(cte_length=20, cte_interval=1)
print("AOA Started")


def Kalman_filter(val, angle, varProcess):
  Pc = angle + varProcess;
  G = Pc/(Pc + std_angle);
  P = (1-G)*Pc;
  Xp = angle;
  Zp = Xp;
  Xe = G*(val-Zp)+Xp; ## "фильтрованное" значение
  return Xe

def minimum(a,n):
    minpos = a.index(min(a))
    return minpos

end_loop_read = 1000

std_angle = 1.6195  ## среднее отклонение для фильтра калмана

list_std = list()
min_std_list = list()
for i in range(1,3000):
    i_tic = 0
    varProcess = i*0.005
    print("Проведение вычислений для Kp = " + str(varProcess))
    angle_arr_1 = list()
    filtered_data_list = list()
    while i_tic<=end_loop_read:  
            try:
                data = rtlsUtil.aoa_results_queue.get(block=True, timeout=0.5)
                i_tic = i_tic + 1
                angle_arr_1.append(data['payload'].angle)
                angle = data['payload'].angle;
                if i_tic == 1:
                    angle_est = 0;
                    filtered_data_list.append(Kalman_filter(angle, angle_est, varProcess))
                else:
                    angle_est = filtered_data_list[i_tic-2];
                    filtered_data_list.append(Kalman_filter(angle, angle_est, varProcess))   
            except queue.Empty:
                i_tic = i_tic +1
                continue
    print("СКО для Kp =" + str(varProcess) +" равно " + str(np.std(filtered_data_list)))
    min_std_list.append(np.std(filtered_data_list))
minpos = minimum(min_std_list,len(min_std_list))
print("Оптимальное значение Kp = " + str(minpos*0.005))

rtlsUtil.aoa_stop()
if rtlsUtil.ble_connected:
    rtlsUtil.ble_disconnect()
    print("Master Disconnected")

rtlsUtil.done()
print("Done")

