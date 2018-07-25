set terminal dumb
set xrange [0:10000]
set yrange [-0.5:1.5]
plot "plot.dat" using 1:2 with lines
pause 0.1
reread
