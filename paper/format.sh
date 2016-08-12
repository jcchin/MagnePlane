
FILES=`find -name '*.tex'`

for file in $FILES; do
	latexindent -w $file
done
