#!/bin/bash
printf "\033c"
pushd ../../../..
if [ -f /opt/ros/melodic/setup.bash     ] ; then source /opt/ros/melodic/setup.bash   ; fi
if [ -f /opt/ros/noetic/setup.bash      ] ; then source /opt/ros/noetic/setup.bash    ; fi
if [ -f ./devel_isolated/setup.bash     ] ; then source ./devel_isolated/setup.bash   ; fi
# if [ -f ./install_isolated/setup.bash ] ; then source ./install_isolated/setup.bash ; fi
# if [ -f ./install/setup.bash          ] ; then source ./install/setup.bash          ; fi

echo -e "run_simu_rms.bash: starting rms emulation\n"

# Start roscore if not yet running
roscore_running=`(ps -elf | grep roscore | grep -v grep | wc -l)`
if [ $roscore_running -lt 1 ] ; then 
  roscore &
  sleep 3
fi

# Start sick_scan emulator
python3 ./src/sick_scan_xd/test/python/sopas_json_test_server.py --tcp_port=2112 --json_file=./src/sick_scan_xd/test/emulator/scandata/20221018_rms_1xxx_ascii_rms2_objects.pcapng.json --scandata_id="sSN LMDradardata" --send_rate=10 --verbosity=2 &
sleep 1

# Start rviz
rosrun rviz rviz -d ./src/sick_scan_xd/test/emulator/config/rviz_emulator_cfg_rms2xxx.rviz --opengl 210 &
sleep 1

# Start sick_scan driver for rms
echo -e "Launching sick_scan sick_rms_2xxx.launch\n"
roslaunch sick_scan sick_rms_2xxx.launch hostname:=127.0.0.1 sw_pll_only_publish:=False &
sleep 1

# Wait for 'q' or 'Q' to exit or until rviz is closed
while true ; do  
  echo -e "rms emulation running. Close rviz or press 'q' to exit..." ; read -t 1.0 -n1 -s key
  if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
  rviz_running=`(ps -elf | grep rviz | grep -v grep | wc -l)`
  if [ $rviz_running -lt 1 ] ; then break ; fi
done

# Shutdown
echo -e "Finishing rms emulation, shutdown ros nodes\n"
rosnode kill -a ; sleep 1
killall sick_generic_caller ; sleep 1
killall sick_scan_emulator ; sleep 1
popd

