#!/bin/bash
if [ $# -eq 1 ]; then
    echo "Starting creation of data..."
    path_rosbags=$1
	path_csv=$2

	cd $path_rosbags
	files=( $( ls . ) )

	for i in "${files[@]}"
	do
		echo "Procesing $i file"
		roslaunch evaluation evaluation.launch rosbag_file:=$path_rosbags/$i csv_directory:=$2 method:=petra
		echo "File $i procesed"
	done
else
    echo "./evaluation.sh <absolute_path_rosbags> <absolute_path_where_save_csv(optional)>"
fi

