#!/bin/bash
#delete spam
find . -regextype posix-extended -regex '.*\.(aux|png|log|pyg|synctex.gz|lof|lot|toc|DS_Store|out|nls|ilg|nlo|bbl|pdf)$' -delete

