#!/bin/bash

# run all python files in parallel under images recursively

cd images/trade_scripts/
find . -regextype posix-egrep -regex '.*(py)$' | xargs -n 1 -P 3 python
cd ../xdsm/
find . -regextype posix-egrep -regex '.*(py)$' | xargs -n 1 -P 3 python 
cd ../..
#build pdf (with bibtex), by defualt do a quick build
"/usr/texbin/pdflatex" -synctex=1 -shell-escape -interaction=nonstopmode heading.tex
if [ "$1" == "-f" ]; then #add -f flag to do full build
	bibtex heading.aux
	makeindex heading.nlo  -s nomencl.ist -o heading.nls
	"/usr/texbin/pdflatex" -synctex=1 -shell-escape -interaction=nonstopmode heading.tex
	"/usr/texbin/pdflatex" -synctex=1 -shell-escape -interaction=nonstopmode heading.tex

	mv heading.pdf magneplane.pdf
fi
#. clean.sh
