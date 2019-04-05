#!/bin/bash
DATE=$(date +%G%m%d_%H:%M:%S)
folder=/home/user/log

clear_logfile()
{
	echo "new" > $folder/record/udp_node.txt
	echo "new" >  $folder/record/plc.txt
	echo "new" >  $folder/record/move.txt
	echo "new" >  $folder/record/moveRound.txt

	echo "new" >  $folder/record/odom.txt
	echo "new" >  $folder/record/odom_prediction.txt
	echo "new" >  $folder/record/pose.txt
	echo "new" >  $folder/record/raw_odom.txt
        echo "new" >  $folder/record/alarm.txt
        echo "new" >  $folder/record/turn.txt
        echo "new" >  $folder/record/charge.log
        echo "new" >  $folder/record/odom_increase.csv

}

add_logfile()
{
	touch  $folder/record/udp_node.txt
	touch  $folder/record/plc.txt
	touch  $folder/record/move.txt
	touch  $folder/record/moveRound.txt

	touch  $folder/record/odom.txt
	touch  $folder/record/odom_prediction.txt
	touch  $folder/record/pose.txt
	touch  $folder/record/raw_odom.txt
        touch  $folder/record/alarm.txt
        touch  $folder/record/turn.txt
        touch  $folder/record/charge.log
        touch  $folder/record/odom_increase.csv
}

if [ ! -d "$folder/record" ]; then
  	mkdir $folder/record
        touch $folder/record/hello.txt
  	echo $DATE >> $folder/record/hello.txt
	echo "record start" >> $folder/record/hello.txt
    
    #add log file 
    add_logfile
    

else 
    #end the record
	echo $DATE >> $folder/record/hello.txt
	echo "end start" >> $folder/record/hello.txt
    #save old file
    var=$(sed -n '1,1p' $folder/record/hello.txt)
    cp -r $folder/record $folder/"record"$var
   
    #new file and clear 
	echo $DATE > $folder/record/hello.txt
	echo "record start" >> $folder/record/hello.txt
    #clear log file 
    clear_logfile

    #delete the old record 3 days before
    rm -rf `find $folder -mmin +4320 -name "record2*"`
fi
