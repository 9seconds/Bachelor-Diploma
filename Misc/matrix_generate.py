#!/usr/bin/env python
#
#       matrix_generate.py
#
#       Copyright 2009 serge arkhipov <serge@aerialsounds.org>
#
#       This program is free software; you can redistribute it and/or modify
#       it under the terms of the GNU General Public License as published by
#       the Free Software Foundation; either version 2 of the License, or
#       (at your option) any later version.
#
#       This program is distributed in the hope that it will be useful,
#       but WITHOUT ANY WARRANTY; without even the implied warranty of
#       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#       GNU General Public License for more details.
#
#       You should have received a copy of the GNU General Public License
#       along with this program; if not, write to the Free Software
#       Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#       MA 02110-1301, USA.

import random
import math

print "\\begin{equation*}\n\\begin{array}{lr}\n\n"

#%------------------------------------
#A_9^0 = \left(\begin{array}{ccc}
#0     &     1     &     0 \\
#-116    &     -2.36   &     116 \\
#2.3  &     0     &     -2.3
#\end{array}\right){,} &
#B_9^0 = \left(\begin{array}{c}
#0 \\
#-42 \\
#0
#\end{array}\right){;} \\

ln = 0
cn = 0
bn = 0
numb = 0
n = range(1,10)

for i in range(1,10):
    rn = random.random()
    numb += random.choice(n)+rn
    bn = random.random()*3+0.1
    ln = ln + random.choice(n)*2 + random.random()
    cn = random.choice(n)/2 + rn*random.random()*14

    print "%------------------------------------"
    print "A_%d^0 = \\left(\\begin{array}{ccc}" % i
    print "0    &    1    &    0 \\\\"
    print "%g    &    %g    &    %g \\\\" % (ln,-cn,ln)
    print "%g    &    0    &    %g" % (bn,bn)
    print "\\end{array}\\right){,} &"
    print "B_%d^0 = \\left(\\begin{array}{c}" % i
    print "0 \\\\"
    print "%g \\\\" % -numb
    print "0"
    print "\\end{array}\\right){;} \\\\"
    print

print "C_{i0} = \\left(\\begin{array}{ccc}\n1 & 0 & 0 \\\\\n0 & 1 & 0\n\\end{array}\\right){,} &\ni \\in \\{1,2,\\ldots,N\}."

print "\\end{array}\n\\end{equation*}"
