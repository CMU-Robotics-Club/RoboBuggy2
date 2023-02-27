#!/bin/bash

# Uses https://github.com/glotlabs/gdrive, must install on target system

FOLDER_ID="1-EfGJie_M0SwkiwtIkeQgErC9Lhv7Cry"
FILES=$(curl -s --get --data-urlencode "includeItemsFromAllDrives=true" --data-urlencode "q='1-EfGJie_M0SwkiwtIkeQgErC9Lhv7Cry' in parents" --data-urlencode "key=$GDRIVE_API_KEY" --data-urlencode "supportsAllDrives=true" --data-urlencode "fields=files(name)" https://www.googleapis.com/drive/v3/files | grep -oh -E '([^\"]*)\.([^\ \"])*')
for file in $(ls /home/nuc/RoboBuggy2/rb_ws/*.{bag,txt,csv});
do
	filebase=$(basename $file)
	if ! grep -q "$filebase" <<< "$FILES"; then
		gdrive files upload --parent $FOLDER_ID $file
		echo $filebase
	fi
done

