#!/bin/bash

for file in $(find . -type f -name \*.py); do
  echo "# Copyright (c) 2023. Lennart Clasmeier. GNU General Public License https://www.gnu.org/licenses/gpl-3.0.html." > copyright-file.py;
  echo "" >> copyright-file.py;
  cat $file >> copyright-file.py;
  mv copyright-file.py $file;
done