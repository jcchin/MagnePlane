Directory

	(main files)
	heading.tex- NASA header/footer, imports packages and subfiles
	sections/introduction.tex - Introduction
	sections/model_overview.tex - Model Overview Section Body + Formatting Subsections
	sections/model_overview/*.tex - Subsections for Model Overview Section
	sections/subsystem_analyses_and_optimizations.tex - Subsystem Analyses and Optimizations Body + Formatting for Subsections
	sections/subsystem_analyses_and_optimizations/*.tex - Subsections for Subsystem Analyses and Optimizations Section
	sections/results.tex - Results Body and Formatting for Subsections
	sections/results/*.tex - Subsections for Results Section
	sections/appendix.tex - Appendix
	sections/conclusion.tex - Conclusion
	localSettings.yaml - settings for latexindent (code formatter/prettifier)
	heading.bib- References
	build.sh - build script

	/images
		-contains all graphics
	/code
		-contains all code snippets
	/unneeded
		-contains archived code
	NASA.cls- NASA LaTeX style sheet
	clean.sh- shell script to delete unnecessary files

Build to PDF using this command:

bash ./build.sh -f


-------------------------------------------
*** Packages ***
-------------------------------------------
BibTex for references (see build script)

\cite{Cengal} to cite a source

http://en.wikipedia.org/wiki/BibTeX#Entry_types
for more info
-------------------------------------------
nomencl for nomenclature (see build script)

\nomenclature{MDAO}{Multi-disciplinary}
\nomenclature{\rho}{Density}

Build with build.sh
Edit .nls to add $$ to math equation nomenclature manually so TexMaker doesn't bark
otherwise can't interperate \dot{W}

http://en.wikibooks.org/wiki/LaTeX/Indexing
for more info
-------------------------------------------
Cleveref for section linking

\cref{app:2} to link to the following appendix section


	\crefalias{section}{appsec}
	\Appendix{Sample Source Code} \label{app:2} 


*Tip*
Label must come after caption for proper figure referencing. 

-------------------------------------------
"mathtools" -for equation 

	\begin{equation*}
	T_{t} = T_{s} * [1 + \frac{\gamma -1}{2} MN^2]
	\end{equation*}

or inline: $<equation>$

-------------------------------------------
"minted" -for code syntax highlighting (see build script)

	\begin{adjustwidth}{-3cm}{-3cm}
	\inputminted[]{python}{code/example1.py}
	\end{adjustwidth} 

Minted Help
http://www.ctan.org/tex-archive/macros/latex/contrib/minted
Read the docs to install:
http://mirrors.ibiblio.org/CTAN/macros/latex/contrib/minted/minted.pdf

if it's still not working check this:
http://tex.stackexchange.com/questions/48018/minted-not-working-on-mac
-------------------------------------------
"siunitx" to format values/units 

\SI{1.0}{\newton\meter}

Help:
http://mirrors.ctan.org/macros/latex/contrib/siunitx/siunitx.pdf