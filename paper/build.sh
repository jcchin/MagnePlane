#!/bin/bash
#build pdf (with bibtex), by defualt do a quick build
"/usr/local/texlive/2016/bin/x86_64-linux/pdflatex" -synctex=1 -shell-escape -interaction=nonstopmode heading.tex
if [ "$1" == "-f" ]; then #add -f flag to do full build
	"/usr/local/texlive/2016/bin/x86_64-linux/bibtex" heading
	"/usr/local/texlive/2016/bin/x86_64-linux/makeindex" heading.nlo  -s nomencl.ist -o heading.nls
	"/usr/local/texlive/2016/bin/x86_64-linux/pdflatex" -synctex=1 -shell-escape -interaction=nonstopmode heading.tex
	"/usr/local/texlive/2016/bin/x86_64-linux/pdflatex" -synctex=1 -shell-escape -interaction=nonstopmode heading.tex

	mv heading.pdf magneplane.pdf
fi
#. clean.sh
