#!/bin/bash
#delete spam
find . -regextype posix-extended -regex '.*\.(aux|log|pyg|synctex.gz|lof|lot|toc|DS_Store|out)$' -delete

