#!/bin/bash

# 获取脚本所在目录
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$SCRIPT_DIR"

echo "Searching for ttyACM devices..."

# 存储设备和序列号的数组
devices=()
serials=()
count=0

# 遍历所有 ttyACM 设备
for dev in /dev/ttyACM*; do
    if [ -e "$dev" ]; then
        # 获取 serial 信息
        # grep ATTRS{serial} 并取第一行，通常是最具体的设备序列号
        info=$(udevadm info -a -n "$dev" | grep 'ATTRS{serial}' | head -n 1)
        
        # 提取序列号
        serial=$(echo "$info" | sed -n 's/.*ATTRS{serial}=="\([^"]*\)".*/\1/p')
        
        if [ -n "$serial" ]; then
            #获取设备编号
            dev_num="${dev#/dev/ttyACM}"
            echo "Found device: $dev (Number: $dev_num) -> Serial: $serial"
            
            devices+=("$dev_num")
            serials+=("$serial")
            ((count++))
        fi
    fi
done

if [ $count -eq 0 ]; then
    echo "No ttyACM devices found."
    exit 0
fi

echo ""
read -p "Enter the ttyACM number you want to select (e.g., 0): " user_choice

# 查找用户选择对应的序列号
selected_serial=""
found=false

for ((i=0; i<count; i++)); do
    if [ "${devices[$i]}" == "$user_choice" ]; then
        selected_serial="${serials[$i]}"
        found=true
        break
    fi
done

if [ "$found" = false ]; then
    echo "Invalid selection: $user_choice"
    exit 1
fi

echo "You selected ttyACM$user_choice with Serial: $selected_serial"

RULES_FILE="arx_can.rules"

if [ -f "$RULES_FILE" ]; then
    printf 'SUBSYSTEM=="tty", ATTRS{idVendor}=="16d0", ATTRS{idProduct}=="117e", ATTRS{serial}=="%s", SYMLINK+="arxcan0"\n' "$selected_serial" > "$RULES_FILE"
    echo "Updated $RULES_FILE with new serial."
else
    printf 'SUBSYSTEM=="tty", ATTRS{idVendor}=="16d0", ATTRS{idProduct}=="117e", ATTRS{serial}=="%s", SYMLINK+="arxcan0"\n' "$selected_serial" > "$RULES_FILE"
    echo "Generated $RULES_FILE with Serial: $selected_serial"
fi

# 运行 set.sh
if [ -f "./set.sh" ]; then
    echo "Running set.sh..."
    ./set.sh
else
    echo "Error: set.sh not found!"
    exit 1
fi
