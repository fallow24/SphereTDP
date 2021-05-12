#!/bin/bash

# This file iterates over all .pose and .3d files in a folder
# If a .3d file is empty (i.e. contains no scans) it and the
# corresponding .pose file is removed. 
# Further all files are renamed such that the numbering is still
# consecutive 

if [ $# -eq 0 ]
  then
    echo "No arguments supplied. Usage:"
    printf "\n\t remove_empty_scans.sh <path> <start_idx> <end_idx> <pts_threshold>\n"
  	exit 1;
fi

PATH=$1
START=$2
END=$3

PTS_THRESHOLD=$4

CURR_WRITE_INDEX=0

echo "Starting with  $START";
echo "Ending with with  $END";

/usr/bin/mkdir "${PATH}reduced";
/usr/bin/rm "${PATH}reduced/*";

for ((i=START;i<=END;i++))
do
	padded_read_count=`printf "%03d" $i`;
	echo "Reading scan${padded_read_count} ...";
	curr_read_file="${PATH}scan${padded_read_count}";
	num_lines=$( /usr/bin/wc -l < "${curr_read_file}.3d" );
	# Delete all scans with less then PTS_THRESHOLD  points
	if [ "$num_lines" -lt "$PTS_THRESHOLD" ];
	then
		echo "... not copying ";
	else
		padded_write_count=`printf "%03d" $CURR_WRITE_INDEX`;
		curr_write_file="${PATH}reduced/scan${padded_write_count}";
		echo "... copying to ${curr_write_file} "
		/usr/bin/cp "${curr_read_file}.3d" "${curr_write_file}.3d";
		/usr/bin/cp "${curr_read_file}.pose" "${curr_write_file}.pose";
		((CURR_WRITE_INDEX++))
	fi
done