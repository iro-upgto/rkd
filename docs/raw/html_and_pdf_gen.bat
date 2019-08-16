@ECHO OFF
"_build/html/index.html"
cd "_build/latex"
pdflatex -interaction=nonstopmode rkd.tex
rkd.pdf