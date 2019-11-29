% Get the latest log file from PX4
log_dir = "/home/jjr/DataFiles/Development/Firmware/build/px4_sitl_default/logs/";
cmd_latest = strcat("find ", log_dir, " -type f -printf ""%T@ %p\n"" | sort -n | cut -d' ' -f 2- | tail -n 1");
[status,latest_log] = system(cmd_latest);

% Replace new-line character from latest-log output
strrep(latest_log, sprintf('\n'), '');

% Copy the latest logfile
cmd_copy = strcat("cp ", latest_log, " ", output_dir, "/", ulog_file);
system(cmd_copy);