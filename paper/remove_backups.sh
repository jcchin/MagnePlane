
FILES=`find . -regex ".*.bak[0-9]*"`

for file in $FILES; do
	rm $file
done
