## Install
```bash
sudo apt install can-utils net-tools
conda create -n arx_control python=3.10
pip install -e .
```

## Can
```bash
./can/search_and_set.sh 
./can/can.sh
```

## Run
```bash
python ./examples/keyboard_teleop.py X5_umi can0
```