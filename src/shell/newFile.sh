#!/bin/bash
DATE=$(date +%m%d_%H:%M:%S)
folder=/home/user/log

if [ ! -d "$folder/record" ]; then
  	mkdir $folder/record
        touch $folder/record/hello.txt
  	echo $(date +%F_%H:%M:%S) >> $folder/record/hello.txt
	echo "This is the latest record" >> $folder/record/hello.txt
else 
	mv $folder/record $folder/"record"$DATE
	mkdir $folder/record
	touch $folder/record/hello.txt
	echo $(date +%F_%H:%M:%S) >> $folder/record/hello.txt
	echo "This is the latest record" >> $folder/record/hello.txt
fi
