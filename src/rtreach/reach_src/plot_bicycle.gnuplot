set terminal png font arial 16 size 500, 600
set output "reach_bicycle.png"

set title "Plot of Reachtubes, reachtime=2.0 s" font "arial,14"
set xlabel "x (m)"
set ylabel "y (m)"

# ranges for pendulum
set autoscale y
set xrange [0:1.7]
set yrange [0.0:1.2]

load "bicycle_initial.gnuplot.txt"
load "bicycle_final.gnuplot.txt"
load "bicycle_intermediate.gnuplot.txt"


plot "bicycle_simulation.dat" using 1:2 with point title ''
plot \
   1/0 lw 4 lc rgb 'blue' with lines t 'Initial', \
   1/0 lw 4 lc rgb 'dark-green' with lines t 'Intermediate', \
   1/0 lw 4 lc rgb 'red' with lines t 'Final'