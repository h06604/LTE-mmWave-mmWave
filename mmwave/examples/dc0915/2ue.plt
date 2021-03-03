set terminal png
set output "rnti1.png"
set title "label vs number"
set xlabel "label"
set ylabel "number"
set xrange[2140:2162]
set yrange[0:22]
plot for[col=1:1] "rnti1.dat" using 1:2 title "ue1" with points
