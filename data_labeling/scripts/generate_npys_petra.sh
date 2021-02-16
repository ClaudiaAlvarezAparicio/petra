#!/bin/bash
if [ $# -eq 2 ]; then
    echo "Starting creation of data..."
    path_rosbags=$1
	path_npys=$2

	cd $path_rosbags
	files=( $( ls . ) )

	for i in "${files[@]}"
	do
		echo "Procesing $i file"
		roslaunch data_labeling data_labeling_with_petra.launch rosbag_file:=$path_rosbags/$i npy_directory:=$path_npys/
		echo "File $i procesed"
	done
else
    echo "./generate_npys.sh <absolute_path_rosbags> <absolute_path_where_save_npys>"
fi

